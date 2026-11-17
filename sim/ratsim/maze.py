"""Mazes: walls of an n x n micromouse maze, the text format of github.com/micromouseonline/mazefiles, a generator.

Cell (x, y): x east, y north, the start (0, 0) in the south-west corner, facing north (the firmware's convention).
"""
import random

N, E, S, W = 1, 2, 4, 8
STEP = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
OPPOSITE = {N: S, E: W, S: N, W: E}


class Maze:
    def __init__(self, size=16):
        self.size = size
        # walls[x][y]: bit mask of the cell's walls; every wall is stored in both cells
        self.walls = [[0] * size for _ in range(size)]
        for i in range(size):
            self.walls[i][0] |= S
            self.walls[i][size - 1] |= N
            self.walls[0][i] |= W
            self.walls[size - 1][i] |= E

    def has(self, x, y, d):
        return bool(self.walls[x][y] & d)

    def set(self, x, y, d, present=True):
        dx, dy = STEP[d]
        nx, ny = x + dx, y + dy
        inside = 0 <= nx < self.size and 0 <= ny < self.size
        if not inside:
            present = True      # the border is always a wall
        for cx, cy, cd in ((x, y, d), (nx, ny, OPPOSITE[d])):
            if 0 <= cx < self.size and 0 <= cy < self.size:
                if present:
                    self.walls[cx][cy] |= cd
                else:
                    self.walls[cx][cy] &= ~cd

    def segments(self):
        """Every wall once: (x, y, 'h') the south wall of row y (y = size: the north border), (x, y, 'v') the
        west wall of column x (x = size: the east border)."""
        n = self.size
        for y in range(n + 1):
            for x in range(n):
                if (y < n and self.has(x, y, S)) or (y == n and self.has(x, n - 1, N)):
                    yield x, y, 'h'
        for x in range(n + 1):
            for y in range(n):
                if (x < n and self.has(x, y, W)) or (x == n and self.has(n - 1, y, E)):
                    yield x, y, 'v'

    # ---- text --------------------------------------------------------------------------------------------
    @classmethod
    def parse(cls, text):
        lines = [l.rstrip('\n') for l in text.splitlines() if l.strip()]
        size = (len(lines) - 1) // 2
        if size < 2 or len(lines) != 2 * size + 1:
            raise ValueError(f"not a maze file: {len(lines)} lines")
        m = cls(size)

        def at(line, col):
            return line[col] if col < len(line) else ' '
        for row in range(size):
            y = size - 1 - row
            top, mid = lines[2 * row], lines[2 * row + 1]
            bottom = lines[2 * row + 2]
            for x in range(size):
                m.set(x, y, N, at(top, 4 * x + 2) not in ' ')
                m.set(x, y, S, at(bottom, 4 * x + 2) not in ' ')
                m.set(x, y, W, at(mid, 4 * x) not in ' ')
                m.set(x, y, E, at(mid, 4 * x + 4) not in ' ')
        return m

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.parse(f.read())

    def text(self):
        n, out = self.size, []
        for row in range(n):
            y = n - 1 - row
            out.append('o' + ''.join(('---' if self.has(x, y, N) else '   ') + 'o' for x in range(n)))
            out.append(''.join(('|' if self.has(x, y, W) else ' ') + '   ' for x in range(n))
                       + ('|' if self.has(n - 1, y, E) else ' '))
        out.append('o' + '---o' * n)
        return '\n'.join(out) + '\n'

    # ---- generator -----------------------------------------------------------------------------------------
    @classmethod
    def generate(cls, seed=None, size=16, loops=0.08, goal=None):
        """A maze by the competition rules: walls on three sides of the start, the goal (default the centre 2x2)
        open inside with one way in, a wall on every post except the goal's centre; `loops`: share of the
        interior walls removed afterwards (alternative routes)."""
        rng = random.Random(seed)
        m = cls(size)
        for x in range(size):
            for y in range(size):
                m.walls[x][y] = N | E | S | W
        if goal is None:
            h = size // 2
            goal = [(h - 1, h - 1), (h, h - 1), (h - 1, h), (h, h)] if size % 2 == 0 else [(h, h)]
        goal_set = set(goal)
        for (x, y) in goal:
            for d, (dx, dy) in STEP.items():
                if (x + dx, y + dy) in goal_set:
                    m.set(x, y, d, False)
        # depth-first carving over the other cells, never through the start's east wall
        start_east = {((0, 0), (1, 0)), ((1, 0), (0, 0))}
        seen = set(goal_set) | {(0, 0)}
        stack = [(0, 0)]
        while stack:
            x, y = stack[-1]
            options = [(d, x + dx, y + dy) for d, (dx, dy) in STEP.items()
                       if 0 <= x + dx < size and 0 <= y + dy < size and (x + dx, y + dy) not in seen
                       and ((x, y), (x + dx, y + dy)) not in start_east]
            if not options:
                stack.pop()
                continue
            d, nx, ny = rng.choice(options)
            m.set(x, y, d, False)
            seen.add((nx, ny))
            stack.append((nx, ny))
        # one way into the goal
        doors = [(x, y, d) for (x, y) in goal for d, (dx, dy) in STEP.items()
                 if 0 <= x + dx < size and 0 <= y + dy < size and (x + dx, y + dy) not in goal_set]
        x, y, d = rng.choice(doors)
        m.set(x, y, d, False)
        # loops: remove interior walls that keep a wall on both their posts
        candidates = [(x, y, d) for x in range(size) for y in range(size) for d in (N, E)
                      if m.has(x, y, d) and 0 <= x + STEP[d][0] < size and 0 <= y + STEP[d][1] < size
                      and (x, y) not in goal_set and (x + STEP[d][0], y + STEP[d][1]) not in goal_set
                      and not (x == 0 and y == 0 and d == E)]
        rng.shuffle(candidates)
        for x, y, d in candidates[:int(loops * len(candidates))]:
            m.set(x, y, d, False)
            if not all(m.post_has_wall(px, py) for px, py in m.posts_of(x, y, d)):
                m.set(x, y, d, True)
        return m

    def posts_of(self, x, y, d):
        if d == N:
            return [(x, y + 1), (x + 1, y + 1)]
        if d == E:
            return [(x + 1, y), (x + 1, y + 1)]
        if d == S:
            return [(x, y), (x + 1, y)]
        return [(x, y), (x, y + 1)]

    def post_has_wall(self, px, py):
        """Post at the corner (px, py) (south-west corner of cell (px, py)): touches at least one wall."""
        n = self.size
        if px in (0, n) or py in (0, n):
            return True
        return (self.has(px - 1, py, E) or self.has(px - 1, py - 1, E)     # the walls north and south of it
                or self.has(px, py - 1, N) or self.has(px - 1, py - 1, N))  # the walls east and west of it
