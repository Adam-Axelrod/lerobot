"""Collect final-frame crops for the by-eye review artifact -> review_data.json (ids blind, shuffled)."""
import base64, glob, json, os, random, sys
import cv2, numpy as np

ROOT = "C:/Users/aa24/PhD/lerobot/"
EV = ROOT + "outputs/eval_auto/"
HALF = 48  # crop +-48 px = +-0.64 mm
UM_PX = 13.40
out_path = sys.argv[1]

def crop_png(img_bgr, c):
    pad = cv2.copyMakeBorder(img_bgr, HALF, HALF, HALF, HALF, cv2.BORDER_CONSTANT, value=(0, 0, 0))
    u, v = int(round(c[0])), int(round(c[1]))
    cr = pad[v: v + 2 * HALF, u: u + 2 * HALF]
    ok, buf = cv2.imencode(".png", cr)
    # sub-pixel position of the target inside the crop
    off = (c[0] - u + HALF, c[1] - v + HALF)
    return "data:image/png;base64," + base64.b64encode(buf.tobytes()).decode(), off

items = []
def add(src, group, png, target, tip, err_um, extra=None):
    img = cv2.imread(png)
    if img is None:
        return
    data, off = crop_png(img, target)
    tipc = None if tip is None else (tip[0] - target[0] + off[0], tip[1] - target[1] + off[1])
    items.append({"src": src, "group": group, "img": data, "target": [round(off[0], 2), round(off[1], 2)],
                  "tip": None if tipc is None else [round(tipc[0], 2), round(tipc[1], 2)],
                  "err_um": None if err_um is None else round(float(err_um), 1), **(extra or {})})

runs = {
    "finish_2026-11_crop256_img_act_finish_zscripted_n15_20261112-161347": "Finish policy, crop only (12 Nov)",
    "finish_2026-11_crop256_js_act_finish_zscripted_n15_20261112-162348": "Finish policy, crop + joints (12 Nov)",
    "finish_2026-11_crop256_js_act_finish_n15_20261111-184654": "Finish policy, own height (11 Nov)",
    "finish_2026-11_crop256_js_act_finish_n15_20261111-184455": "Finish policy, own height (11 Nov)",
}
for d in sorted(glob.glob(EV + "*_refine_*")):
    runs[os.path.basename(d)] = "Finish policy + learned refinement (12 Nov)"
handover = {"finish_2026-11_crop256_img_act_handover_zscripted_n15_20261112-170439": [4, 8, 5],
            "finish_2026-11_crop256_js_act_handover_zscripted_n15_20261112-171139": [8]}
for run, label in runs.items():
    for l in open(EV + run + "/results.jsonl"):
        r = json.loads(l)
        add(f"{run}:ep{r['episode']}", label, f"{EV}{run}/ep{r['episode']:03d}/final.png", r["target_px"], r.get("vision_tip_px"), r.get("vision_error_um"))
for run, eps in handover.items():
    for l in open(EV + run + "/results.jsonl"):
        r = json.loads(l)
        if r["episode"] in eps:
            add(f"{run}:ep{r['episode']}", "Finish policy from the pre-touch point (12 Nov)", f"{EV}{run}/ep{r['episode']:03d}/final.png", r["target_px"], r.get("vision_tip_px"), r.get("vision_error_um"))
# teacher demos: 10 spanning the error distribution
fin = os.path.expanduser("~/.cache/huggingface/lerobot/AdamAxelrod/basler_stage_touch_finish_2026-11/")
meta = [json.loads(x) for x in open(fin + "autocollect_episodes.jsonl") if x.strip()]
meta = [m for m in meta if m.get("final") and m["final"].get("detected_px") and m.get("final_error_um") is not None]
meta.sort(key=lambda m: m["final_error_um"])
for k in np.linspace(0, len(meta) - 1, 10).round().astype(int):
    m = meta[k]
    add(f"finish_dataset:ep{m['episode_index']}", "Scripted teacher demo (11 Nov)", f"{fin}autocollect/raw/ep{m['episode_index']:04d}/final.png", m["target_px"], m["final"]["detected_px"], m["final_error_um"])

random.Random(7).shuffle(items)
for i, it in enumerate(items):
    it["id"] = f"F{i + 1:02d}"
json.dump({"half_px": HALF, "um_per_px": UM_PX, "items": items}, open(out_path, "w"))
print(len(items), "items;", sum(len(i["img"]) for i in items) // 1024, "KB of images")
