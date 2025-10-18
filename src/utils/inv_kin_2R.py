from math import pi, sin, cos, atan2, sqrt, degrees
import matplotlib.pyplot as plt

# Dati
l1=l2=0.5
l3=0.0          # metti la lunghezza reale dell’ultimo link
x,y=0.1,0.25
al=pi/2

# Polso
ax = x - l3*cos(al)
ay = y - l3*sin(al)

# IK 2R
r2 = ax*ax + ay*ay
c2 = (r2 - l1*l1 - l2*l2)/(2*l1*l2)
if abs(c2) > 1: raise ValueError("Target irraggiungibile")
s2 = sqrt(1 - c2*c2)          # gomito-su; usa -s2 per gomito-giù
q2 = atan2(s2, c2)
q1 = atan2(ay, ax) - atan2(l2*s2, l1 + l2*c2)
q3 = al - q1 - q2

print(f"q1: {degrees(q1):.2f}°, q2: {degrees(q2):.2f}°")

# FK per i punti
x0,y0 = 0.0,0.0
x1,y1 = x0 + l1*cos(q1),         y0 + l1*sin(q1)
x2,y2 = x1 + l2*cos(q1+q2),      y1 + l2*sin(q1+q2)
x3,y3 = x2 + l3*cos(q1+q2+q3),   x2 + l3*sin(q1+q1+q3)

# Plot
fig, ax = plt.subplots()
ax.plot([x0,x1],[y0,y1], marker="o")
ax.plot([x1,x2],[y1,y2], marker="o")
ax.plot([x2,x3],[y2,y3], marker="o")
L = 0.15
ax.set_aspect("equal"); ax.grid(True); ax.legend(); ax.set_xlabel("x"); ax.set_ylabel("y")
plt.show()