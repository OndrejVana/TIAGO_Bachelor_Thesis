# -*- coding: utf-8 -*-
from __future__ import print_function, division
import heapq
import time


class SearchResult(object):
    def __init__(self, success, path, cost, expands, message):
        self.success = success
        self.path = path
        self.cost = cost
        self.expands = expands
        self.message = message


def reconstruct(came_from, goal):
    out = [goal]
    cur = goal

    while cur in came_from:
        cur = came_from[cur]
        out.append(cur)

    out.reverse()
    return out


def _current_time():
    return time.time()


def _push_open(open_heap, f_score, counter, state):
    heapq.heappush(open_heap, (f_score, counter, state))


def _pop_open(open_heap):
    return heapq.heappop(open_heap)


def _make_failure_result(message, expands):
    return SearchResult(False, [], float("inf"), expands, message)


def _make_success_result(came, state, g_score, expands, message):
    return SearchResult(
        True,
        reconstruct(came, state),
        g_score[state],
        expands,
        message
    )


def _expand_successors(s, succ, g_score, came, open_heap, w, heuristic, counter):
    """
    Expand one state and update OPEN / g-values.

    Returns:
        updated_counter
    """
    for ns, step_cost in succ(s):
        ng = g_score[s] + step_cost

        if ns not in g_score or ng < g_score[ns]:
            g_score[ns] = ng
            came[ns] = s
            counter += 1

            f_score = ng + w * heuristic(ns)
            _push_open(open_heap, f_score, counter, ns)

    return counter


def weighted_astar(start, is_goal, succ, heuristic, w, time_limit_s):
    t_end = _current_time() + time_limit_s

    open_heap = []
    g_score = {start: 0.0}
    came = {}
    closed = set()

    expands = 0
    counter = 0

    _push_open(open_heap, w * heuristic(start), counter, start)

    while open_heap and _current_time() < t_end:
        _, _, s = _pop_open(open_heap)

        if s in closed:
            continue

        closed.add(s)
        expands += 1

        if is_goal(s):
            return _make_success_result(
                came, s, g_score, expands, "weighted A* success"
            )

        counter = _expand_successors(
            s=s,
            succ=succ,
            g_score=g_score,
            came=came,
            open_heap=open_heap,
            w=w,
            heuristic=heuristic,
            counter=counter,
        )

    if _current_time() >= t_end:
        return _make_failure_result(
            "weighted_astar timeout without solution",
            expands
        )

    return _make_failure_result(
        "weighted_astar exhausted without solution",
        expands
    )


def _run_weighted_astar_iteration(start, is_goal, succ, heuristic, eps, total_start_time, total_time_s):
    """
    Run one weighted A* iteration for the current epsilon.
    """
    elapsed = _current_time() - total_start_time
    remaining = total_time_s - elapsed

    return weighted_astar(
        start,
        is_goal,
        succ,
        heuristic,
        w=eps,
        time_limit_s=max(0.01, remaining)
    )


def _update_best_result(best, result):
    """
    Keep the lower-cost successful result.
    """
    if not result.success:
        return best

    if best is None or result.cost < best.cost:
        return result

    return best


def _schedule_finished(eps, eps_end):
    return eps < eps_end


def _no_solution_result(elapsed, total_time_s, finished_schedule):
    if elapsed >= total_time_s and not finished_schedule:
        return SearchResult(
            False, [], float("inf"), 0,
            "anytime schedule timeout without solution"
        )

    return SearchResult(
        False, [], float("inf"), 0,
        "anytime schedule exhausted without solution"
    )


def _success_result(best, elapsed, total_time_s, finished_schedule):
    if elapsed >= total_time_s and not finished_schedule:
        return SearchResult(
            True,
            best.path,
            best.cost,
            best.expands,
            "anytime schedule timeout with incumbent solution"
        )

    return SearchResult(
        True,
        best.path,
        best.cost,
        best.expands,
        "anytime schedule success"
    )


def eps_schedule_search(start, is_goal, succ, heuristic, eps_start, eps_end, eps_step, total_time_s):
    """
    Repeated Weighted A* with a decreasing epsilon schedule.
    Each iteration restarts from scratch with a lower epsilon and keeps the best solution found.
    """
    best = None
    t0 = _current_time()
    eps = eps_start

    while eps >= eps_end and (_current_time() - t0) < total_time_s:
        result = _run_weighted_astar_iteration(
            start=start,
            is_goal=is_goal,
            succ=succ,
            heuristic=heuristic,
            eps=eps,
            total_start_time=t0,
            total_time_s=total_time_s,
        )

        best = _update_best_result(best, result)
        eps -= eps_step

    elapsed = _current_time() - t0
    finished_schedule = _schedule_finished(eps, eps_end)

    if best is None:
        return _no_solution_result(elapsed, total_time_s, finished_schedule)

    return _success_result(best, elapsed, total_time_s, finished_schedule)