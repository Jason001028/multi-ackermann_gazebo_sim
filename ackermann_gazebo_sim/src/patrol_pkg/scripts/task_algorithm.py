#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
task_algorithm.py
多无人车任务分配与调度管理节点（ROS1 / move_base / actionlib）

支持三种算法（通过 ROS 参数 ~algorithm 切换）：
  greedy    —— 纯贪婪策略（Baseline）
  km        —— 基于方向权重的 KM 匹配算法
  mtd_hcga  —— 宏观战略牵引的分布式多层协同博弈算法（MTD-HCGA）

启动示例：
  rosrun patrol_pkg task_algorithm.py _algorithm:=mtd_hcga _alpha:=1.0 _beta:=0.05 \
      _d_macro:=[1.0,-1.0] _prescreening_radius:=50.0 \
      _tasks_file:=$(rospack find patrol_pkg)/scripts/intent_tasks.json
"""

import os
import json
import math
import time
import csv
import threading

import rospy
import actionlib
import numpy as np
from scipy.optimize import linear_sum_assignment

from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

# ──────────────────────────────────────────────────────────────
# 全局常量
# ──────────────────────────────────────────────────────────────
CAR_NAMES = ['car1', 'car2', 'car3']

# 仿真初始位置（与 launch 文件保持一致）
CAR_INIT_POSES = {
    'car1': (0.0,   13.0),
    'car2': (14.0, -11.0),
    'car3': (-2.0,  -2.0),
}

# 各车可视化颜色 (R, G, B, A)
CAR_COLORS = {
    'car1': (1.0, 0.2, 0.2, 1.0),   # 红
    'car2': (0.2, 1.0, 0.2, 1.0),   # 绿
    'car3': (0.2, 0.4, 1.0, 1.0),   # 蓝
}

# 车辆导航状态
ST_IDLE       = 'IDLE'
ST_NAVIGATING = 'NAVIGATING'


# ──────────────────────────────────────────────────────────────
# 数据类：任务
# ──────────────────────────────────────────────────────────────
class Task(object):
    def __init__(self, task_id, x, y, base_priority):
        self.task_id       = task_id
        self.x             = float(x)
        self.y             = float(y)
        self.base_priority = int(base_priority)
        self.assigned_to   = None
        self.completed     = False
        self.complete_time = None
        self.fail_count    = 0      # 累计失败次数
        self.blacklisted   = False  # 超过阈值后跳过


# ──────────────────────────────────────────────────────────────
# 数据类：车辆运行时状态
# ──────────────────────────────────────────────────────────────
class CarState(object):
    def __init__(self, name, init_x, init_y):
        self.name           = name
        self.x              = init_x    # 当前 AMCL 定位坐标
        self.y              = init_y
        self.state          = ST_IDLE
        self.current_task   = None      # 正在执行的 Task 对象
        self.total_distance = 0.0       # 累计行驶里程（m）
        self._last_ox       = init_x    # 上一帧 odom 坐标（用于积分）
        self._last_oy       = init_y
        self._odom_ready    = False
        # 任务分配序列：[(x, y), ...]，按派发顺序追加，用于 C_total / η_align 计算
        self.task_sequence  = []


# ──────────────────────────────────────────────────────────────
# 全局监视器：指标采集与结算
# ──────────────────────────────────────────────────────────────
class GlobalMonitor(object):
    def __init__(self, tasks, algorithm_name):
        self.tasks      = tasks
        self.algo       = algorithm_name
        self.start_time = None
        self._finalized = False

        # 找出 base_priority 最高的 Top-3 任务 ID（战略响应指标）
        sorted_t = sorted(tasks, key=lambda t: t.base_priority, reverse=True)
        self.top3_ids = set(t.task_id for t in sorted_t[:3])
        self.strategic_response_time = None   # 秒，相对 start_time

    def check_strategic_response(self):
        """Top-3 任务全部完成时记录战略响应耗时"""
        if self.strategic_response_time is not None:
            return
        if all(t.completed for t in self.tasks if t.task_id in self.top3_ids):
            self.strategic_response_time = time.time() - self.start_time
            rospy.loginfo("[Monitor] Top-3 战略任务全部完成，SRT=%.2fs" %
                          self.strategic_response_time)

    def all_done(self):
        return all(t.completed for t in self.tasks)

    # ── 新增：三个理论评价指标 ────────────────────────────────
    @staticmethod
    def _euclidean(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _calc_makespan_theoretical(self, cars, avg_speed=1.0):
        """
        T_makespan = max_i( sum of path lengths in car_i's sequence / avg_speed )
        起点取车辆初始位置，后续节点取 task_sequence 中的坐标。
        avg_speed 单位 m/s，默认 1.0。
        """
        end_times = []
        for name in CAR_NAMES:
            car = cars[name]
            seq = car.task_sequence
            if not seq:
                end_times.append(0.0)
                continue
            init_pos = CAR_INIT_POSES[name]
            waypoints = [init_pos] + seq
            path_len = sum(
                self._euclidean(waypoints[k], waypoints[k + 1])
                for k in range(len(waypoints) - 1)
            )
            end_times.append(path_len / avg_speed)
        return max(end_times)

    def _calc_c_total(self, cars):
        """
        C_total = sum_i sum_k c(p_{i,k-1}, p_{i,k})
        c(·) 用欧式距离替代（无法直接获取 TEB 真实里程时的标准替代）。
        起点取车辆初始位置。
        """
        total = 0.0
        for name in CAR_NAMES:
            car = cars[name]
            seq = car.task_sequence
            if not seq:
                continue
            init_pos = CAR_INIT_POSES[name]
            waypoints = [init_pos] + seq
            total += sum(
                self._euclidean(waypoints[k], waypoints[k + 1])
                for k in range(len(waypoints) - 1)
            )
        return total

    def _calc_eta_align(self, cars, d_macro):
        """
        η_align = (1/3) * sum_i cos_sim(v_{i,term}, d_macro)
        v_{i,term} = 最后一个任务点的坐标向量（相对原点）。
        若某车无任务序列，该车贡献 0。
        """
        d_norm = np.linalg.norm(d_macro)
        if d_norm < 1e-9:
            return 0.0
        d_unit = d_macro / d_norm

        cosines = []
        for name in CAR_NAMES:
            seq = cars[name].task_sequence
            if not seq:
                cosines.append(0.0)
                continue
            v = np.array(seq[-1], dtype=float)
            v_norm = np.linalg.norm(v)
            if v_norm < 1e-9:
                cosines.append(0.0)
            else:
                cosines.append(float(np.dot(v / v_norm, d_unit)))
        return float(np.mean(cosines))

    def finalize(self, cars, d_macro=None):
        """所有任务完成后调用：打印结算面板并写 CSV"""
        if self._finalized:
            return
        try:
            makespan  = time.time() - self.start_time
            distances = [cars[n].total_distance for n in CAR_NAMES]
            total_d   = sum(distances)
            variance  = float(np.var(distances))
            srt       = self.strategic_response_time if self.strategic_response_time else makespan

            if d_macro is None:
                d_macro = np.array([1.0, -1.0])
            t_makespan_th = self._calc_makespan_theoretical(cars)
            c_total       = self._calc_c_total(cars)
            eta_align     = self._calc_eta_align(cars, d_macro)

            self._print_panel(makespan, total_d, variance, srt, distances,
                              t_makespan_th, c_total, eta_align)
            self._write_csv(makespan, total_d, variance, srt, distances)
            self._write_theory_csv(t_makespan_th, c_total, eta_align)
            self._finalized = True  # 只有全部成功才标记
        except Exception as e:
            rospy.logerr("[Monitor] finalize 异常: %s" % str(e))
            import traceback
            rospy.logerr(traceback.format_exc())

    def _print_panel(self, makespan, total_d, variance, srt, distances,
                     t_makespan_th, c_total, eta_align):
        W    = 68
        sep  = "=" * W
        sep2 = "-" * W
        rospy.loginfo("\n" + sep)
        rospy.loginfo("  实验结算面板  |  Algorithm: %-20s" % self.algo)
        rospy.loginfo(sep2)
        rospy.loginfo("  %-42s %s" % ("指标", "数值"))
        rospy.loginfo(sep2)
        rospy.loginfo("  %-42s %.3f  s"  % ("Makespan          (集群总完工时间)",  makespan))
        rospy.loginfo("  %-42s %.3f  m"  % ("Total_Distance    (全局行驶总里程)",  total_d))
        rospy.loginfo("  %-42s %.4f m2"  % ("Workload_Variance (负载均衡方差)",    variance))
        rospy.loginfo("  %-42s %.3f  s"  % ("Strategic_Response_Time (战略响应)",  srt))
        rospy.loginfo(sep2)
        rospy.loginfo("  %-42s %.4f  s"  % ("T_makespan_theoretical (理论完工)",   t_makespan_th))
        rospy.loginfo("  %-42s %.4f  m"  % ("C_total (全局机动总代价)",             c_total))
        rospy.loginfo("  %-42s %.4f"     % ("η_align (终端战略对齐度)",             eta_align))
        rospy.loginfo(sep2)
        for i, name in enumerate(CAR_NAMES):
            rospy.loginfo("  %-42s %.3f  m" % ("%s 行驶里程" % name, distances[i]))
        rospy.loginfo(sep + "\n")

    def _write_csv(self, makespan, total_d, variance, srt, distances):
        csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'experiment_metrics.csv')
        exists = os.path.isfile(csv_path)
        with open(csv_path, 'a') as f:
            w = csv.writer(f)
            if not exists:
                w.writerow(['Algorithm', 'Makespan_s', 'Total_Distance_m',
                            'Workload_Variance_m2', 'Strategic_Response_Time_s',
                            'car1_dist_m', 'car2_dist_m', 'car3_dist_m', 'Timestamp'])
            w.writerow([
                self.algo,
                round(makespan,  3),
                round(total_d,   3),
                round(variance,  4),
                round(srt,       3),
                round(distances[0], 3),
                round(distances[1], 3),
                round(distances[2], 3),
                time.strftime('%Y-%m-%d %H:%M:%S'),
            ])
        rospy.loginfo("[Monitor] 指标已追加写入: %s" % csv_path)

    def _write_theory_csv(self, t_makespan_th, c_total, eta_align):
        """将三个理论评价指标追加写入 experiment_metrics_log.csv"""
        csv_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'experiment_metrics_log.csv')
        exists = os.path.isfile(csv_path)
        with open(csv_path, 'a') as f:
            w = csv.writer(f)
            if not exists:
                w.writerow(['Timestamp', 'Algorithm',
                            'T_makespan_theoretical_s',
                            'C_total_m',
                            'eta_align'])
            w.writerow([
                time.strftime('%Y-%m-%d %H:%M:%S'),
                self.algo,
                round(t_makespan_th, 4),
                round(c_total,       4),
                round(eta_align,     4),
            ])
        rospy.loginfo("[Monitor] 理论指标已追加写入: %s" % csv_path)


# ──────────────────────────────────────────────────────────────
# MTD-HCGA 算法：宏观战略牵引的分布式多层协同博弈
# ──────────────────────────────────────────────────────────────
class MTD_HCGA_Allocator(object):
    """
    三阶段分配器：
      Phase 0 — 预筛选（距离阈值过滤潜在任务簇）
      Phase 1 — 启发式拍卖预分配（各车独立竞标，无冲突直接分配）
      Phase 2 — KM 二部图匹配（处理冲突车辆与剩余任务，完整矩形矩阵）
    """

    def __init__(self, cars, d_macro, alpha, beta, prescreening_radius=50.0):
        self.cars                = cars
        self.alpha               = float(alpha)
        self.beta                = float(beta)
        self.prescreening_radius = float(prescreening_radius)
        # 一次性归一化，后续方法直接使用 d_unit，避免循环内重复计算和除零风险
        d = np.array(d_macro, dtype=float)
        d_norm = np.linalg.norm(d)
        self.d_unit = d / d_norm if d_norm > 1e-9 else np.array([1.0, 0.0])

    def allocate(self, idle_cars, unassigned_tasks):
        """
        主入口：返回 {car_name: Task} 映射。
        idle_cars        — list[str]，空闲车辆名称列表
        unassigned_tasks — list[Task]，未分配任务列表
        """
        if not idle_cars or not unassigned_tasks:
            return {}
        candidate_tasks = self._phase0_prescreening(idle_cars, unassigned_tasks)
        rospy.loginfo("[MTD-HCGA] Phase0: %d/%d 任务进入候选簇" %
                      (len(candidate_tasks), len(unassigned_tasks)))
        no_conflict, conflict_cars, conflict_tasks = self._phase1_auction(
            idle_cars, candidate_tasks)
        rospy.loginfo("[MTD-HCGA] Phase1: %d 无冲突分配, %d 辆车进入Phase2" %
                      (len(no_conflict), len(conflict_cars)))
        km_assignments = self._phase2_km_resolve(conflict_cars, conflict_tasks)
        rospy.loginfo("[MTD-HCGA] Phase2: KM 解决 %d 个分配" % len(km_assignments))
        final = {}
        final.update(no_conflict)
        final.update(km_assignments)
        return final

    def _phase0_prescreening(self, idle_cars, unassigned_tasks):
        """
        保留至少一辆空闲车在 prescreening_radius 内的任务。
        若无任何任务满足条件，回退到全量任务列表（保证算法不空转）。
        """
        candidate = []
        for task in unassigned_tasks:
            for car_name in idle_cars:
                car = self.cars[car_name]
                if math.sqrt((car.x - task.x) ** 2 + (car.y - task.y) ** 2) \
                        <= self.prescreening_radius:
                    candidate.append(task)
                    break
        return candidate if candidate else list(unassigned_tasks)

    def get_teb_heuristic_cost(self, car_name, task):
        """
        TEB 路径代价桩：返回欧氏距离。
        TODO: 当 TEB 全局规划器接口可用时，替换此方法以获取真实路径代价。
        """
        car = self.cars[car_name]
        return math.sqrt((car.x - task.x) ** 2 + (car.y - task.y) ** 2)

    def _compute_bid(self, car_name, task):
        """
        bid = path_cost - alpha * cos_sim(dir_to_task, d_macro) - priority_bonus
        bid 越小 → 该车对该任务越有竞争力（拍卖最小化代价）。

        path_cost      : TEB 启发式代价（当前为欧氏距离桩）
        cos_sim        : 任务方向与宏观方向的余弦相似度，使用预归一化的 self.d_unit
        priority_bonus : base_priority * 2.0，高优先级任务降低竞标代价
        """
        path_cost = self.get_teb_heuristic_cost(car_name, task)
        car = self.cars[car_name]
        dx, dy = task.x - car.x, task.y - car.y
        dist = math.sqrt(dx * dx + dy * dy) + 1e-9
        cos_sim = float(np.dot([dx / dist, dy / dist], self.d_unit))
        return path_cost - self.alpha * cos_sim - task.base_priority * 2.0

    def _phase1_auction(self, idle_cars, candidate_tasks):
        """
        每辆车对候选任务计算竞标代价，选出各自最优任务（最小 bid）。
        返回 (no_conflict_assignments, conflict_cars, conflict_tasks)：
          no_conflict_assignments — dict[car_name, Task]，无冲突直接分配
          conflict_cars           — list[str]，需进入 Phase2 的车辆
          conflict_tasks          — list[Task]，需进入 Phase2 的任务
        """
        if not candidate_tasks:
            return {}, list(idle_cars), []
        bids = {cn: {t.task_id: self._compute_bid(cn, t) for t in candidate_tasks}
                for cn in idle_cars}
        car_best = {}
        for cn in idle_cars:
            best = min(candidate_tasks, key=lambda t: bids[cn][t.task_id])
            car_best[cn] = best
            rospy.logdebug("[MTD-HCGA] %s 首选 T%d  bid=%.3f" % (
                cn, best.task_id, bids[cn][best.task_id]))
        return self._detect_conflicts(car_best, bids, candidate_tasks)

    def _detect_conflicts(self, car_best_task, bids, candidate_tasks):
        """
        统计每任务被多少车选为首选：
          唯一选择者 → 无冲突，直接分配（no_conflict）
          多车竞争   → 所有竞争者进入 Phase2（conflict_cars）

        返回的 conflict_tasks（即 remaining）包含两类任务：
          1. 被多辆车竞争的冲突任务
          2. 无人首选的剩余任务（确保 Phase2 有足够任务可供匹配）
        两类任务统一交给 Phase2 的 KM 算法处理。
        assigned_ids 代表"Phase1 已成功确权的任务"（唯一竞标者获得）。
        """
        task_map  = {t.task_id: t for t in candidate_tasks}
        wanted_by = {}
        for cn, task in car_best_task.items():
            wanted_by.setdefault(task.task_id, []).append(cn)

        no_conflict, conflict_cars, conflict_task_ids = {}, [], set()
        for tid, cars in wanted_by.items():
            if len(cars) == 1:
                no_conflict[cars[0]] = task_map[tid]
            else:
                conflict_cars.extend(cars)
                conflict_task_ids.add(tid)

        # assigned_ids：Phase1 已成功确权的任务（唯一竞标者获得）
        # remaining：冲突任务 + 无人首选任务，全部送入 Phase2
        assigned_ids = {t.task_id for t in no_conflict.values()}
        remaining    = [t for t in candidate_tasks if t.task_id not in assigned_ids]
        return no_conflict, list(set(conflict_cars)), remaining

    def _phase2_km_resolve(self, conflict_cars, conflict_tasks):
        """
        对冲突车辆与剩余任务构建完整 N_cars×N_tasks 收益矩阵，
        调用 linear_sum_assignment 在完整解空间中求最优分配。
        不截断任务列表，避免因方向优先级截断而丢失近距离任务。
        score[i,j] = alpha * dir_priority(task_j) - beta * dist(car_i, task_j)
        """
        if not conflict_cars or not conflict_tasks:
            return {}
        n_cars  = len(conflict_cars)
        n_tasks = len(conflict_tasks)

        def dir_p(t):
            return float(np.dot([t.x, t.y], self.d_unit)) + t.base_priority * 5.0

        # 构建完整矩形收益矩阵（n_cars × n_tasks），不截断
        score = np.zeros((n_cars, n_tasks))
        for i, cn in enumerate(conflict_cars):
            car = self.cars[cn]
            for j, task in enumerate(conflict_tasks):
                d = math.sqrt((car.x - task.x) ** 2 + (car.y - task.y) ** 2)
                score[i, j] = self.alpha * dir_p(task) - self.beta * d

        # linear_sum_assignment 原生支持矩形矩阵，最小化负收益 = 最大化收益
        row_ind, col_ind = linear_sum_assignment(-score)

        assignments = {}
        for i, j in zip(row_ind, col_ind):
            cn, task = conflict_cars[i], conflict_tasks[j]
            assignments[cn] = task
            rospy.loginfo(
                "[MTD-HCGA/KM] %s -> T%d  score=%.3f  p_dir=%.2f  dist=%.2fm" % (
                    cn, task.task_id, score[i, j], dir_p(task),
                    math.sqrt((self.cars[cn].x - task.x) ** 2 +
                              (self.cars[cn].y - task.y) ** 2)))
        return assignments


# ──────────────────────────────────────────────────────────────
# 主节点类
# ──────────────────────────────────────────────────────────────
class TaskAlgorithmNode(object):
    def __init__(self):
        rospy.init_node('task_algorithm', anonymous=False)

        # ── 读取 ROS 参数 ──────────────────────────────────────
        self.algo    = rospy.get_param('~algorithm', 'km')       # 'greedy' | 'km' | 'mtd_hcga'
        self.alpha   = float(rospy.get_param('~alpha',  1.0))    # 方向权重系数 α
        self.beta    = float(rospy.get_param('~beta',   0.05))   # 距离惩罚系数 β
        d_raw        = rospy.get_param('~d_macro', [1.0, -1.0])  # 宏观调度方向向量
        self.d_macro = np.array(d_raw, dtype=float)
        self.prescreening_radius = float(rospy.get_param('~prescreening_radius', 50.0))
        tasks_file   = rospy.get_param(
            '~tasks_file',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'intent_tasks.json'))

        self._lock = threading.Lock()
        self._task_fail_limit = int(rospy.get_param('~task_fail_limit', 3))
        self._nav_timeout     = float(rospy.get_param('~nav_timeout', 120.0))  # 单次导航超时(s)
        self._goal_send_time  = {}   # {car_name: time.time()} 记录每次 send_goal 的时刻

        # ── 加载任务 ───────────────────────────────────────────
        self.tasks = self._load_tasks(tasks_file)
        rospy.loginfo("[Init] 加载 %d 个任务，算法: %s，d_macro=%s" %
                      (len(self.tasks), self.algo, self.d_macro.tolist()))

        # ── 初始化车辆状态 ─────────────────────────────────────
        self.cars = {n: CarState(n, *CAR_INIT_POSES[n]) for n in CAR_NAMES}

        # ── 全局监视器 ─────────────────────────────────────────
        self.monitor = GlobalMonitor(self.tasks, self.algo)

        # ── MTD-HCGA 分配器（仅在 algo == 'mtd_hcga' 时使用）──
        self._mtd_hcga = MTD_HCGA_Allocator(
            self.cars, self.d_macro, self.alpha, self.beta,
            self.prescreening_radius)

        # ── actionlib 客户端（每辆车独立，用于发送目标并监听结果）
        self.ac = {}
        for name in CAR_NAMES:
            ac = actionlib.SimpleActionClient('/%s/move_base' % name, MoveBaseAction)
            rospy.loginfo("[Init] 等待 /%s/move_base action server..." % name)
            ac.wait_for_server(rospy.Duration(15.0))
            self.ac[name] = ac
        rospy.loginfo("[Init] 所有 move_base action server 已就绪")

        # ── 可视化发布者 ───────────────────────────────────────
        self.marker_pub = rospy.Publisher(
            '/task_assignment_markers', MarkerArray, queue_size=1)

        # ── 订阅 odom（里程积分）和 amcl_pose（精确定位）──────
        for name in CAR_NAMES:
            rospy.Subscriber('/%s/odom' % name, Odometry,
                             self._odom_cb, callback_args=name)
            rospy.Subscriber('/%s/amcl_pose' % name, PoseWithCovarianceStamped,
                             self._amcl_cb, callback_args=name)

        # ── 等待定位稳定后启动分配 ─────────────────────────────
        rospy.sleep(2.0)
        self.monitor.start_time = time.time()
        rospy.loginfo("[Init] 开始任务分配，算法: %s" % self.algo)
        with self._lock:
            self._trigger_assignment()

        # ── 可视化定时器（2 Hz）───────────────────────────────
        rospy.Timer(rospy.Duration(0.5), self._publish_markers)

        # ── 兜底定时器：防止 done_cb 未触发导致结算卡住 ──────
        rospy.Timer(rospy.Duration(2.0), self._watchdog_check)

        # ── 诊断定时器（每 10s 打印一次任务与车辆状态）────────
        rospy.Timer(rospy.Duration(10.0), self._diag_print)

    # ──────────────────────────────────────────────────────────
    # 工具函数
    # ──────────────────────────────────────────────────────────
    @staticmethod
    def _load_tasks(path):
        with open(path, 'r') as f:
            data = json.load(f)
        return [Task(d['task_id'], d['x'], d['y'], d['base_priority']) for d in data]

    @staticmethod
    def _dist(x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def _dir_priority(self, task):
        """
        方向修正优先级：
          P = dot([task.x, task.y], d_macro_normalized) + base_priority * 5
        点积越大 → 任务越符合宏观调度方向 → 优先级越高
        """
        d_norm = self.d_macro / (np.linalg.norm(self.d_macro) + 1e-9)
        return float(np.dot([task.x, task.y], d_norm)) + task.base_priority * 5.0

    # ──────────────────────────────────────────────────────────
    # 订阅回调
    # ──────────────────────────────────────────────────────────
    def _odom_cb(self, msg, name):
        """里程计积分：累加相邻帧位移，不加锁（float 写入是原子的）"""
        car = self.cars[name]
        ox  = msg.pose.pose.position.x
        oy  = msg.pose.pose.position.y
        if car._odom_ready:
            dx = ox - car._last_ox
            dy = oy - car._last_oy
            car.total_distance += math.sqrt(dx * dx + dy * dy)
        else:
            car._odom_ready = True
        car._last_ox = ox
        car._last_oy = oy

    def _amcl_cb(self, msg, name):
        """用 AMCL 定位更新车辆坐标（供分配算法使用）"""
        car   = self.cars[name]
        car.x = msg.pose.pose.position.x
        car.y = msg.pose.pose.position.y

    # ──────────────────────────────────────────────────────────
    # actionlib done 回调工厂
    # ──────────────────────────────────────────────────────────
    def _make_done_cb(self, car_name):
        """为每辆车生成独立的 done_cb 闭包，避免 lambda 共享变量问题"""
        def done_cb(state, result):  # result unused — required by actionlib API signature
            with self._lock:
                car  = self.cars[car_name]
                task = car.current_task
                if task is None:
                    return

                if state == GoalStatus.SUCCEEDED:
                    task.completed     = True
                    task.complete_time = time.time() - self.monitor.start_time
                    car.state          = ST_IDLE
                    car.current_task   = None
                    rospy.loginfo("[%s] 完成任务 T%d  (t=%.1fs)" %
                                  (car_name, task.task_id, task.complete_time))
                    self.monitor.check_strategic_response()
                    if self.monitor.all_done():
                        skipped = [t for t in self.tasks if t.blacklisted]
                        if skipped:
                            rospy.logwarn("[Monitor] %d 个任务因多次失败被跳过: %s" % (
                                len(skipped), [t.task_id for t in skipped]))
                        rospy.loginfo("[Monitor] 全部任务完成，生成结算面板...")
                        self.monitor.finalize(self.cars, self.d_macro)
                        return
                else:
                    # ABORTED / PREEMPTED：归还任务，重新调度
                    task.fail_count += 1
                    rospy.logwarn("[%s] 任务 T%d 失败(state=%d)，累计失败 %d 次" %
                                  (car_name, task.task_id, state, task.fail_count))
                    if task.fail_count >= self._task_fail_limit:
                        task.blacklisted = True
                        task.completed   = True   # 标记为完成以跳过，避免阻塞全局结算
                        rospy.logwarn("[%s] 任务 T%d 失败超过 %d 次，永久跳过" %
                                      (car_name, task.task_id, self._task_fail_limit))
                    else:
                        task.assigned_to = None
                    car.state        = ST_IDLE
                    car.current_task = None

                self._trigger_assignment()
        return done_cb

    # ──────────────────────────────────────────────────────────
    # 任务分配入口（在 _lock 内调用）
    # ──────────────────────────────────────────────────────────
    def _trigger_assignment(self):
        """根据算法将未分配任务派发给空闲车辆"""
        unassigned = [t for t in self.tasks
                      if not t.completed and t.assigned_to is None]
        idle_cars  = [n for n in CAR_NAMES
                      if self.cars[n].state == ST_IDLE]
        if not unassigned or not idle_cars:
            # 检查是否所有非黑名单任务都已完成
            if self.monitor.all_done():
                skipped = [t for t in self.tasks if t.blacklisted]
                if skipped:
                    rospy.logwarn("[Monitor] %d 个任务因多次失败被跳过: %s" % (
                        len(skipped), [t.task_id for t in skipped]))
                rospy.loginfo("[Monitor] 全部任务完成（含跳过），生成结算面板...")
                self.monitor.finalize(self.cars, self.d_macro)
            return

        if self.algo == 'greedy':
            self._assign_greedy(idle_cars, unassigned)
        elif self.algo == 'mtd_hcga':
            self._assign_mtd_hcga(idle_cars, unassigned)
        else:
            self._assign_km(idle_cars, unassigned)

    # ──────────────────────────────────────────────────────────
    # 算法一：纯贪婪策略
    # ──────────────────────────────────────────────────────────
    def _assign_greedy(self, idle_cars, unassigned):
        """
        每辆空闲车独立选择距离自己最近的未分配任务。
        忽略宏观方向权重，仅基于欧氏距离，作为 Baseline。
        """
        available = list(unassigned)
        for car_name in idle_cars:
            if not available:
                break
            car  = self.cars[car_name]
            best = min(available,
                       key=lambda t: self._dist(car.x, car.y, t.x, t.y))
            available.remove(best)
            rospy.loginfo("[Greedy] %s -> T%d  dist=%.2fm" % (
                car_name, best.task_id,
                self._dist(car.x, car.y, best.x, best.y)))
            self._send_goal(car_name, best)

    # ──────────────────────────────────────────────────────────
    # 算法二：基于方向权重的 KM 匹配算法
    # ──────────────────────────────────────────────────────────
    def _assign_km(self, idle_cars, unassigned):
        """
        构建收益矩阵 Score(i,j) = α·P_directional(j) - β·Distance(i,j)
        用 scipy.optimize.linear_sum_assignment（匈牙利算法）求最优分配，
        使整个簇的总收益最大化。
        每轮只分配 min(空闲车数, 剩余任务数) 个任务。
        """
        n = min(len(idle_cars), len(unassigned))

        # 按方向得分降序排列，优先考虑高价值任务
        sorted_tasks = sorted(unassigned,
                              key=self._dir_priority, reverse=True)
        cand_tasks = sorted_tasks[:n]
        cand_cars  = idle_cars[:n]

        # 构建 n×n 收益矩阵
        score = np.zeros((n, n))
        for i, car_name in enumerate(cand_cars):
            car = self.cars[car_name]
            for j, task in enumerate(cand_tasks):
                d = self._dist(car.x, car.y, task.x, task.y)
                score[i, j] = self.alpha * self._dir_priority(task) - self.beta * d

        # linear_sum_assignment 最小化成本 → 取负值转为最大化收益
        row_ind, col_ind = linear_sum_assignment(-score)

        for i, j in zip(row_ind, col_ind):
            car_name = cand_cars[i]
            task     = cand_tasks[j]
            car      = self.cars[car_name]
            rospy.loginfo(
                "[KM] %s -> T%d  score=%.3f  p_dir=%.2f  dist=%.2fm" % (
                    car_name, task.task_id, score[i, j],
                    self._dir_priority(task),
                    self._dist(car.x, car.y, task.x, task.y)))
            self._send_goal(car_name, task)

    # ──────────────────────────────────────────────────────────
    # 算法三：MTD-HCGA 宏观战略牵引的分布式多层协同博弈
    # ──────────────────────────────────────────────────────────
    def _assign_mtd_hcga(self, idle_cars, unassigned):
        """调用 MTD_HCGA_Allocator.allocate() 获取分配映射，对每个分配调用 _send_goal。"""
        assignments = self._mtd_hcga.allocate(idle_cars, unassigned)
        for car_name, task in assignments.items():
            rospy.loginfo("[MTD-HCGA] 最终分配: %s -> T%d (%.1f, %.1f)" % (
                car_name, task.task_id, task.x, task.y))
            self._send_goal(car_name, task)

    # ──────────────────────────────────────────────────────────
    # 发送导航目标
    # ──────────────────────────────────────────────────────────
    def _send_goal(self, car_name, task):
        """标记任务已分配，通过 actionlib 发送 MoveBaseGoal"""
        task.assigned_to        = car_name
        car                     = self.cars[car_name]
        car.current_task        = task
        car.state               = ST_NAVIGATING
        car.task_sequence.append((task.x, task.y))   # 记录分配序列，供理论指标计算
        self._goal_send_time[car_name] = time.time()  # 记录发送时刻，供超时检测

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = '%s/map' % car_name
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = task.x
        goal.target_pose.pose.position.y = task.y
        goal.target_pose.pose.orientation.w = 1.0

        self.ac[car_name].send_goal(goal, done_cb=self._make_done_cb(car_name))
        rospy.loginfo("[Send] %s -> T%d (%.1f, %.1f)" % (
            car_name, task.task_id, task.x, task.y))

    # ──────────────────────────────────────────────────────────
    # RViz 可视化
    # ──────────────────────────────────────────────────────────
    def _publish_markers(self, event=None):
        """
        每 0.5s 发布一次 MarkerArray：
          - LINE_STRIP：车辆当前位置 → 目标点连线（各车不同颜色）
          - SPHERE：车辆当前位置标记
          - TEXT_VIEW_FACING：任务编号与优先级文字标签
        """
        ma = MarkerArray()
        now = rospy.Time.now()

        for idx, name in enumerate(CAR_NAMES):
            car = self.cars[name]
            r, g, b, a = CAR_COLORS[name]
            color = ColorRGBA(r, g, b, a)

            # ── 连线：车辆 → 目标点 ──────────────────────────
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp    = now
            line.ns              = 'assignment_lines'
            line.id              = idx
            line.type            = Marker.LINE_STRIP
            line.action          = Marker.ADD
            line.scale.x         = 0.12
            line.color           = color
            line.lifetime        = rospy.Duration(1.0)
            if car.state == ST_NAVIGATING and car.current_task:
                p0 = Point(car.x, car.y, 0.15)
                p1 = Point(car.current_task.x, car.current_task.y, 0.15)
                line.points = [p0, p1]
            ma.markers.append(line)

            # ── 球体：车辆当前位置 ───────────────────────────
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp    = now
            sphere.ns              = 'car_positions'
            sphere.id              = idx + 10
            sphere.type            = Marker.SPHERE
            sphere.action          = Marker.ADD
            sphere.pose.position   = Point(car.x, car.y, 0.3)
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.5
            sphere.color           = ColorRGBA(r, g, b, 0.85)
            sphere.lifetime        = rospy.Duration(1.0)
            ma.markers.append(sphere)

        # ── 文字标签：所有任务点 ─────────────────────────────
        for task in self.tasks:
            txt = Marker()
            txt.header.frame_id = 'map'
            txt.header.stamp    = now
            txt.ns              = 'task_labels'
            txt.id              = task.task_id + 30
            txt.type            = Marker.TEXT_VIEW_FACING
            txt.action          = Marker.ADD
            txt.pose.position   = Point(task.x, task.y, 0.9)
            txt.pose.orientation.w = 1.0
            txt.scale.z         = 0.45
            txt.text            = "T%d(P%d)" % (task.task_id, task.base_priority)
            txt.lifetime        = rospy.Duration(1.0)

            if task.completed:
                txt.color = ColorRGBA(0.5, 0.5, 0.5, 0.4)   # 灰色=已完成
            elif task.assigned_to:
                r2, g2, b2, _ = CAR_COLORS[task.assigned_to]
                txt.color = ColorRGBA(r2, g2, b2, 1.0)       # 对应车辆颜色=执行中
            else:
                txt.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)   # 白色=待分配
            ma.markers.append(txt)

        self.marker_pub.publish(ma)

    def _diag_print(self, _event=None):
        """每 10s 打印一次任务与车辆状态，便于排查卡死原因"""
        lines = ["[Diag] ---- 状态快照 ----"]
        for name in CAR_NAMES:
            car = self.cars[name]
            ac_state = self.ac[name].get_state()
            task_info = ("T%d" % car.current_task.task_id) if car.current_task else "None"
            lines.append("  %s: state=%s  task=%s  ac_state=%d  pos=(%.1f,%.1f)" % (
                name, car.state, task_info, ac_state, car.x, car.y))
        pending = [t for t in self.tasks if not t.completed]
        for t in pending:
            lines.append("  Task T%d: assigned_to=%s  fail=%d  blacklisted=%s" % (
                t.task_id, t.assigned_to, t.fail_count, t.blacklisted))
        if not pending:
            if self.monitor._finalized:
                lines.append("  所有任务已完成，finalize 已成功执行")
            else:
                lines.append("  所有任务已完成，finalize 尚未执行（可能异常）")
        rospy.logwarn("\n".join(lines))

    def _watchdog_check(self, _event=None):
        """兜底检查：所有任务已完成但 finalize 未触发时强制结算；单次导航超时时强制 cancel"""
        if self.monitor._finalized:
            return
        with self._lock:
            # 超时检测：cancel 卡死的导航目标，归还任务重新调度
            now = time.time()
            for name in CAR_NAMES:
                car = self.cars[name]
                if car.state != ST_NAVIGATING or car.current_task is None:
                    continue
                elapsed = now - self._goal_send_time.get(name, now)
                if elapsed > self._nav_timeout:
                    task = car.current_task
                    rospy.logwarn(
                        "[Watchdog] %s 导航 T%d 已超时 %.0fs，强制 cancel 并归还任务" %
                        (name, task.task_id, elapsed))
                    self.ac[name].cancel_goal()
                    task.fail_count += 1
                    if task.fail_count >= self._task_fail_limit:
                        task.blacklisted = True
                        task.completed   = True
                        rospy.logwarn("[Watchdog] T%d 失败超过 %d 次，永久跳过" %
                                      (task.task_id, self._task_fail_limit))
                    else:
                        task.assigned_to = None
                    car.state        = ST_IDLE
                    car.current_task = None
                    self._goal_send_time.pop(name, None)
                    self._trigger_assignment()

            if self.monitor.all_done():
                rospy.logwarn("[Watchdog] 检测到全部任务完成但未结算，强制触发 finalize")
                self.monitor.finalize(self.cars, self.d_macro)

    def spin(self):
        rospy.spin()


# ──────────────────────────────────────────────────────────────
# 入口
# ──────────────────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        node = TaskAlgorithmNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
