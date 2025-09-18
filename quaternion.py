import numpy as np

class Quaternion:
    def __init__(self, q):
        q = np.array(q, dtype=float)
        if q.shape != (4,):
            raise ValueError("Quaternion must be a 4-element array [w, x, y, z]")
        self.array = q

    def __repr__(self):
        return f"Quaternion({self.q})"

    def normalize(self):
        self.q = self.q / np.linalg.norm(self.q)
        return self

    def conjugate(self):
        w, x, y, z = self.q
        return Quaternion(w, -x, -y, -z)
    
    def vec(self):
        w, x, y, z =  self.q
        return np.array([x, y, z])
    
    def __add__(self, other):
        return Quaternion(self.q + other.q)
    
def quat_mult(q1, q2):

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return Quaternion(w, x, y, z)