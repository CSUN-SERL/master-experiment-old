
class HumanFinder:

    def _on_segment(p, q, r):
        if q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]):
            return True

        return False

    def _find_orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0
        if val > 0:
            return 1
        return 2

    def _lines_intersect(a1, a2, b1, b2):
        o1 = _find_orientation(a1, a2, b1)
        o2 = _find_orientation(a1, a2, b2)
        o3 = _find_orientation(b1, b2, a1)
        o4 = _find_orientation(b1, b2, a2)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and (_on_segment(a1, b1, a2) or _on_segment(a1, b2, a2) or _on_segment(b1, a1, b2) or _on_segment(b1, a2, b2)):
            return True

        return False

    def wall_intersects_view_to_human(human_x, human_y, wall_tl, wall_br, robot_x, robot_y):
        #Get wall diagonals
        wall_tr = (wall_br[0], wall_tl[1])
        wall_bl = (wall_tl[0], wall_br[1])
        wall_diag_1 = (wall_tl, wall_br)
        wall_diag_2 = (wall_tr, wall_bl)
        wall_diags = [wall_diag_1, wall_diag_2]
        human_line = ((robot_x, robot_y), (human_x, human_y))
        #For each wall diagonal
        for wall_diag in wall_diags:
            if _lines_intersect(human_line[0], human_line[1], wall_diag[0], wall_diag[1]):
                return True
        return False
