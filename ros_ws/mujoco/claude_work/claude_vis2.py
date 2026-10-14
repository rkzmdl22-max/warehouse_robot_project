import cv2, numpy as np
from claude_vis import show, pump, ann
F = cv2.FONT_HERSHEY_SIMPLEX
def cam_info(d, cid, r, truth):
    c = r.get("center")
    if c is None: return ["RGB-D: no detection"]
    p = d.cam_xpos[cid]
    return ["bottle center (x,y,z)=(%.3f,%.3f,%.3f) m | camera pos=(%.3f,%.3f,%.3f) m" % (c[0], c[1], c[2], p[0], p[1], p[2]),
            "camera->bottle distance=%.3f m | error vs sim truth=%.1f mm (eval only)" % (np.linalg.norm(c - p), np.linalg.norm(c - truth) * 1000)]
def ann_side(title, r, lines, x=0, y=0):
    raw = np.ascontiguousarray(r["rgb"].copy()); img = raw.copy()
    mk = r["mask"]; v, u = np.nonzero(mk); c = r.get("center")
    if len(u):
        cn, _ = cv2.findContours(mk.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cn, -1, (0, 255, 0), 2)
        cv2.rectangle(img, (int(u.min()), int(v.min())), (int(u.max()), int(v.max())), (0, 255, 0), 1)
        cv2.drawMarker(img, (int(u.mean()), int(v.mean())), (255, 255, 0), cv2.MARKER_CROSS, 18, 2)
    cv2.putText(img, title, (8, 22), F, 0.6, (255, 255, 0), 2)
    cv2.putText(img, "BOTTLE DETECTED (mask %d px)" % r["npix"] if c is not None else "NO DETECTION", (8, 46), F, 0.55, (0, 255, 0), 2)
    if c is not None: cv2.putText(img, "visible width=%.1fmm" % (r["diam"] * 1000), (8, 68), F, 0.45, (0, 255, 0), 1)
    cv2.putText(raw, "RAW (no overlay)", (8, 22), F, 0.6, (255, 255, 255), 2)
    top = np.hstack([raw, img]); bar = np.zeros((26 * max(1, len(lines)) + 12, top.shape[1], 3), np.uint8)
    for i, t in enumerate(lines): cv2.putText(bar, t, (10, 26 * (i + 1)), F, 0.5, (255, 255, 255), 1)
    show(img, title, x, y)
    return np.ascontiguousarray(np.vstack([top, bar]))
