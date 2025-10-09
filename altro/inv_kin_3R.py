from math import pi, sin, cos, atan2, sqrt, degrees
import matplotlib.pyplot as plt

# Dati (esempio)
l1, l2, l3 = 0.5, 0.5, 0.5      # lunghezze link
x, y = 0.1, 0.25                # target TCP
al = pi/4                       # orientazione TCP desiderata

# Punto polso = target - l3 lungo alpha
wx = x - l3*cos(al)
wy = y - l3*sin(al)

# IK 2R su (wx, wy)
r2 = wx*wx + wy*wy
c2 = (r2 - l1*l1 - l2*l2) / (2*l1*l2)
if abs(c2) > 1:
    raise ValueError("Target irraggiungibile")
s2 = sqrt(max(0.0, 1 - c2*c2))   # elbow-up; usa -s2 per elbow-down
q2 = atan2(s2, c2)
q1 = atan2(wy, wx) - atan2(l2*s2, l1 + l2*c2)

# Polso: impone l'orientazione
q3 = al - q1 - q2

print(f"q1: {degrees(q1):.2f}°, q2: {degrees(q2):.2f}°, q3: {degrees(q3):.2f}°")

# FK per i punti
x0, y0 = 0.0, 0.0
x1, y1 = x0 + l1*cos(q1),         y0 + l1*sin(q1)
x2, y2 = x1 + l2*cos(q1+q2),      y1 + l2*sin(q1+q2)
x3, y3 = x2 + l3*cos(q1+q2+q3),   y2 + l3*sin(q1+q2+q3)

# Plot
fig, ax = plt.subplots()
ax.plot([x0,x1],[y0,y1], marker="o", label="link1")
ax.plot([x1,x2],[y1,y2], marker="o", label="link2")
ax.plot([x2,x3],[y2,y3], marker="o", label="link3/TCP")
ax.plot([x],[y], "x", label="target")
ax.set_aspect("equal"); ax.grid(True); ax.legend()
ax.set_xlabel("x"); ax.set_ylabel("y")
plt.show()
