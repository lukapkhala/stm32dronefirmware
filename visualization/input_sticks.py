import serial
import pygame
import re
import sys

# -------- Serial Setup --------
PORT = 'COM5'
BAUD = 115200
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Failed to open {PORT} @ {BAUD}: {e}")
    sys.exit(1)

# -------- Pygame Setup --------
pygame.init()
WIDTH, HEIGHT = 900, 560
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("RC Transmitter (Mode 2 • AETR)")
font = pygame.font.SysFont("Arial", 18)
clock = pygame.time.Clock()

# -------- RC Range --------
RC_MIN, RC_MAX = 1000, 2000
MID = int((RC_MIN + RC_MAX) / 2)

# -------- Layout --------
PAD = 10
left_box  = pygame.Rect(90,  90, 260, 260)   # Yaw/Throttle
right_box = pygame.Rect(550, 90, 260, 260)   # Roll/Pitch
STICK_RADIUS = 14

# Default channels (CH1..CH6)
ch = [MID, MID, MID, MID, MID, MID]

# -------- Helpers --------
def norm(val, in_min, in_max, out_min, out_max):
    if in_max == in_min:
        return out_min
    t = (val - in_min) / (in_max - in_min)
    t = max(0, min(1, t))
    return int(out_min + t * (out_max - out_min))

def draw_gimbal(rect, x_val, y_val, title, labels):
    pygame.draw.rect(screen, (210, 210, 210), rect, 2)
    cx, cy = rect.center
    pygame.draw.line(screen, (90, 90, 90), (rect.left, cy), (rect.right, cy), 1)
    pygame.draw.line(screen, (90, 90, 90), (cx, rect.top), (cx, rect.bottom), 1)

    x_min, x_max = rect.left + PAD, rect.right - PAD
    y_min, y_max = rect.top + PAD, rect.bottom - PAD

    sx = norm(x_val, RC_MIN, RC_MAX, x_min, x_max)
    sy = norm(y_val, RC_MIN, RC_MAX, y_max, y_min)
    pygame.draw.circle(screen, (0, 200, 255), (sx, sy), STICK_RADIUS)

    title_surf = font.render(title, True, (255, 255, 255))
    screen.blit(title_surf, (rect.left, rect.top - 26))
    xlab = font.render(labels[0], True, (180, 180, 180))
    ylab = font.render(labels[1], True, (180, 180, 180))
    screen.blit(xlab, (rect.centerx - xlab.get_width() // 2, rect.bottom + 6))
    screen.blit(ylab, (rect.right + 8, rect.centery - ylab.get_height() // 2))

def parse_line(line):
    m = re.findall(r"CH(\d+):\s*(\d+)", line)
    if not m:
        return None
    vals = {}
    for idx, v in m:
        idx = int(idx)
        if 1 <= idx <= 16:
            vals[idx] = int(v)
    return vals

# -------- Main Loop --------
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # ---- Read serial ----
    try:
        line = ser.readline().decode(errors="ignore").strip()
    except Exception:
        line = ""
    vals = parse_line(line)
    if vals:
        for i in range(1, min(7, max(vals.keys()) + 1)):  # now up to CH6
            if i in vals:
                ch[i - 1] = max(RC_MIN, min(RC_MAX, vals[i]))

    # ---- Draw ----
    screen.fill((30, 30, 30))

    # Left stick = Yaw/Throttle
    draw_gimbal(left_box, x_val=ch[3], y_val=ch[2],
                title="Left Stick — Yaw / Throttle",
                labels=("Yaw (CH4)", "Thr (CH3)"))

    # Right stick = Roll/Pitch
    draw_gimbal(right_box, x_val=ch[0], y_val=ch[1],
                title="Right Stick — Roll / Pitch",
                labels=("Roll (CH1)", "Pitch (CH2)"))

    # Text readout
    readout = (f"CH1(Roll): {ch[0]}   CH2(Pitch): {ch[1]}   "
               f"CH3(Throttle): {ch[2]}   CH4(Yaw): {ch[3]}   "
               f"CH5: {ch[4]}   CH6: {ch[5]}")
    txt = font.render(readout, True, (255, 255, 255))
    screen.blit(txt, (WIDTH//2 - txt.get_width()//2, 380))

    # CH5 slider
    x0, y0, w = 270, 440, 360
    pygame.draw.rect(screen, (120, 120, 120), (x0, y0, w, 6))
    knob_x = norm(ch[4], RC_MIN, RC_MAX, x0, x0 + w)
    pygame.draw.circle(screen, (0, 255, 120), (knob_x, y0 + 3), 8)
    lab = font.render(f"CH5: {ch[4]}", True, (200, 200, 200))
    screen.blit(lab, (x0 + w + 16, y0 - 10))

    # CH6 slider
    y0_6 = 480
    pygame.draw.rect(screen, (120, 120, 120), (x0, y0_6, w, 6))
    knob_x6 = norm(ch[5], RC_MIN, RC_MAX, x0, x0 + w)
    pygame.draw.circle(screen, (0, 255, 180), (knob_x6, y0_6 + 3), 8)
    lab6 = font.render(f"CH6: {ch[5]}", True, (200, 200, 200))
    screen.blit(lab6, (x0 + w + 16, y0_6 - 10))

    pygame.display.flip()
    clock.tick(60)

ser.close()
pygame.quit()
