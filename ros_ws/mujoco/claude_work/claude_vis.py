import cv2, numpy as np
import os; ON = [os.environ.get("VIS", "1") == "1"]; N = [0]
def show(img, name, x, y):
    if not ON[0]: return
    try:
        cv2.namedWindow(name); cv2.moveWindow(name, x, y); cv2.imshow(name, cv2.cvtColor(cv2.resize(img, (480, 360)), cv2.COLOR_RGB2BGR)); cv2.waitKey(1)
    except Exception as e: ON[0] = False; print("camera window unavailable:", str(e)[:60])
def pump():
    N[0] += 1
    if ON[0] and N[0] % 40 == 0:
        try: cv2.waitKey(1)
        except Exception: ON[0] = False
def ann(title, r, x, y):
    img = np.ascontiguousarray(r["rgb"].copy()); mk = r["mask"]; v, u = np.nonzero(mk)
    img[mk] = (0.55 * img[mk] + 0.45 * np.array([0, 255, 0])).astype(np.uint8)
    if len(u): cv2.rectangle(img, (int(u.min()), int(v.min())), (int(u.max()), int(v.max())), (0, 255, 0), 2); cv2.drawMarker(img, (int(u.mean()), int(v.mean())), (255, 255, 0), cv2.MARKER_CROSS, 16, 2)
    c = r.get("center"); F = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, title, (8, 22), F, 0.6, (255, 255, 0), 2); cv2.putText(img, "BOTTLE DETECTED" if c is not None else "NO DETECTION", (8, 46), F, 0.55, (0, 255, 0), 2)
    if c is not None: cv2.putText(img, "D=%.1fmm C=(%.3f,%.3f,%.3f)" % (r["diam"] * 1000, c[0], c[1], c[2]), (8, 68), F, 0.45, (0, 255, 0), 1)
    show(img, title, x, y); return img
