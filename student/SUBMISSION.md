# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Nhật Thắng
- MSSV: 2A202602727
- Email: junifoca1331@gmail.com
- Link repo (fork): https://github.com/nhat-thang/K4-L2L3-DAY23-NguyenNhatThang-2A202602727-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`):

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: 0.9701, 0.7004, 519 / 16 / 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1503 m, 502, 11.3436, 0, 239, 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1359 m, 502, 9.2668, 0, 239, 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: hai mode có cùng `matches` (502), `ghost_track_frames` (0), `missed_gt_frames` (239) và `mean_confirmed_tracks`, vì chỉ lượt LiDAR quyết định tạo/xác nhận/xoá track nên tập track confirmed giống hệt nhau. Khác biệt chỉ nằm ở độ chính xác vị trí: update camera thêm thông tin góc ngang/dọc, kéo trạng thái EKF gần GT hơn nên `sum_sq_err` giảm từ 11.34 xuống 9.27 và RMSE giảm 0.150 → 0.136 m (tốt hơn ~0.014 m, nằm trong ngưỡng không xấu hơn 0.05 m). 239 frame GT bị miss chủ yếu do recall detector 0.70 (222 FN): xe không được detect thì không có track, camera không bù được vì không được sinh track.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?
   LiDAR đo trực tiếp vị trí 3D `z = (x, y, z)` (mét), `R = diag(0.1², 0.1², 0.1²)`, mô hình đo
   tuyến tính `h(x) = R_vs·p + t` nên `H` là ma trận 3×6 hằng (phần vận tốc bằng 0). Camera đo
   pixel `z = (u, v)`, `R = diag(5², 5²)` px², mô hình pinhole `u = c_i − f_i·y_s/x_s`,
   `v = c_j − f_j·z_s/x_s` là phi tuyến (chia cho độ sâu) nên `H` là Jacobian 2×6 tính tại `x`
   hiện tại. Camera không đo được độ sâu, chỉ ràng buộc hướng nhìn; LiDAR ràng buộc cả 3 trục.
2. Vì sao cần gating Mahalanobis trước khi gán?
   `d² = γᵀ S⁻¹ γ` chuẩn hoá residual theo bất định `S = HPHᵀ + R`, nên so sánh được giữa các
   track có `P` khác nhau và giữa mét/pixel. Với γ Gauss, `d²` theo phân phối χ² bậc = số chiều
   đo (3 cho LiDAR, 2 cho camera); cổng `χ²(0.995)` loại các cặp quá khó xảy ra. Nếu không gate,
   greedy sẽ ghép cả cặp sai ở xa (một xe với detection của xe khác/FP), EKF update kéo track lệch
   và track đáng lẽ miss lại được cộng score.
3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   Track-then-fuse: mỗi frame trong `run_lab.run` gọi `KF.predict` một lần cho mọi track, rồi
   `associate_and_update(..., lidar_sensor)` (AssocL), sau đó mới `associate_and_update(...,
   camera_sensor)` (AssocC) trên cùng tập track. Đo hai sensor không được gộp trước; chúng update
   tuần tự một EKF. Trên log: `grade_run_lidar.log` và `grade_run_fused.log` có `confirmed`,
   `matches`, `ghosts`, `misses` giống hệt nhau từng frame, chỉ `sum_sq_err` khác — camera chỉ tinh
   chỉnh trạng thái của track đã có từ LiDAR.
4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   Innovation camera `γ = z − h(x)` sẽ có độ lệch hệ thống (trung bình khác 0, cùng dấu qua nhiều
   frame và nhiều xe) thay vì nhiễu quanh 0; `d²` trung bình tăng lên trên mức kỳ vọng (≈ 2 với
   đo 2 chiều). Hậu quả: nhiều cặp bị gate loại (ít camera update), các cặp còn lại kéo track lệch
   khỏi vị trí LiDAR, residual LiDAR ngay sau đó cũng tăng và RMSE fused có thể xấu hơn LiDAR-only.
5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   Khi `meas_list` rỗng thì không suy ra sensor từ `meas.sensor` được, nhưng vẫn phải gọi
   `manage_tracks` để biết đây là lượt LiDAR: mọi track trong FOV bị tính miss (−1/window) và có
   thể bị xoá. Nếu bỏ qua frame rỗng, track ma sẽ sống mãi. LiDAR đo vị trí 3D đầy đủ và có detector
   thật, nên là nguồn đáng tin về sự tồn tại; camera trong lab là tâm hộp 2D không có độ sâu,
   không đủ để khởi tạo trạng thái 3D, và FOV hẹp hơn — nếu camera trừ score thì track ngoài/rìa
   ảnh bị xoá oan. Vì vậy camera chỉ cập nhật `x`, `P`.
6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   Track mới: `score = 1/6`, `initialized`. Mỗi lượt LiDAR: hit → `score = min(score + 1/6, 1)`
   và track chưa confirmed thành `tentative`; miss trong FOV → `score −= 1/6`. Xác nhận khi
   `score > 0.8` (strict). Đã `confirmed` thì không bị hạ trạng thái khi miss; chỉ bị xoá khi
   `score < 0.6`. Track chưa confirmed bị xoá khi `score ≤ 0`. Ngoài ra xoá bất kể score khi
   `P[0,0] > 9` hoặc `P[1,1] > 9` (σ vị trí ngang > 3 m).

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (Claude Opus 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): sắp xếp thư mục dữ liệu và cài môi trường (CP0); implement Part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`); debug lỗi encoding cp1252 trên Windows và lỗi `float()` với ma trận 1×1; soạn nháp phần tóm tắt và 6 câu giải thích (đã đọc lại và chỉnh theo hiểu biết của mình).
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` (128 passed, không failed/xfailed); chạy `fusion-run-lab --fusion compare --seed 0` trên frame 0–198, số liệu lấy từ `metrics.json`; đối chiếu công thức EKF/pinhole/χ² với docs/HUONG_DAN_KY_THUAT.md §2.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
