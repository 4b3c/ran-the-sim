"""A tiny RGB canvas written as PPM, so the prototypes need nothing outside the standard library."""


class Canvas:
    def __init__(self, w, h, bg=(255, 255, 255)):
        self.w, self.h = w, h
        self.px = bytearray(bytes(bg) * (w * h))

    def dot(self, x, y, c, r=0):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                xi, yi = int(x) + dx, int(y) + dy
                if 0 <= xi < self.w and 0 <= yi < self.h:
                    i = 3 * (yi * self.w + xi)
                    self.px[i:i + 3] = bytes(c)

    def line(self, x0, y0, x1, y1, c, r=0):
        n = int(max(abs(x1 - x0), abs(y1 - y0))) + 1
        for k in range(n + 1):
            t = k / n
            self.dot(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, c, r)

    def save_ppm(self, path):
        with open(path, "wb") as f:
            f.write(b"P6 %d %d 255\n" % (self.w, self.h))
            f.write(self.px)
