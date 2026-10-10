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
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: số ghép (502), ghost (0), miss (239) và số track confirmed trung bình của 2 mode y hệt nhau. Lý do là camera không được tạo/xoá track hay cộng trừ score, nên danh sách track ở 2 mode là một. Cái khác duy nhất là vị trí: có thêm update camera thì track nằm sát xe thật hơn một chút, sum_sq_err giảm từ 11.34 còn 9.27, RMSE từ 0.150 xuống 0.136 m. Còn 239 frame miss thì phần lớn do detector bỏ sót (recall chỉ 0.70, 222 FN). Xe nào LiDAR không detect được thì không có track, camera cũng không cứu được.

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
   LiDAR cho luôn vị trí 3D của tâm hộp, `z = (x, y, z)` tính bằng mét, `R` là đường chéo 0.1².
   Hàm đo chỉ là đổi hệ toạ độ nên tuyến tính, `H` là ma trận 3×6 cố định (3 cột vận tốc = 0).
   Camera thì cho pixel `z = (u, v)`, `R` là đường chéo 5² (px²). Để ra pixel phải chiếu pinhole,
   có chia cho độ sâu `x_s`, nên không tuyến tính, phải tính Jacobian tại trạng thái hiện tại.
   Camera không biết xe cách bao xa, chỉ biết xe nằm theo hướng nào.
2. Vì sao cần gating Mahalanobis trước khi gán?
   Khoảng cách Euclid thường không tính đến độ bất định. Mahalanobis `d² = γᵀ S⁻¹ γ` chia residual
   cho `S`, nên track mới (P to) được phép lệch nhiều hơn track đã ổn định, và so được cả mét lẫn
   pixel. `d²` theo phân phối χ² với bậc tự do bằng số chiều đo, nên lấy ngưỡng χ² 99.5% để loại
   các cặp vô lý. Không gate thì greedy có thể ghép track với detection của xe khác hoặc FP,
   track bị kéo lệch mà còn được tính là hit.
3. Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.
   Track-then-fuse. Trong `run_lab.py` mỗi frame predict tất cả track 1 lần, update bằng LiDAR
   trước (AssocL), xong mới tới camera (AssocC) trên chính các track đó. Không có bước gộp đo
   LiDAR với camera trước khi track. Nhìn log cũng thấy: so `grade_run_lidar.log` với
   `grade_run_fused.log` thì 199 frame đều có `confirmed`, `matches`, `ghosts`, `misses` giống
   nhau, chỉ `sum_sq_err` khác. Tức là camera chỉ chỉnh lại vị trí track mà LiDAR đã tạo.
4. Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?
   Residual camera sẽ bị lệch về một phía, kiểu frame nào xe nào cũng lệch vài chục pixel cùng
   chiều, chứ không còn dao động quanh 0 nữa. `d²` trung bình cao hơn mức bình thường (khoảng 2
   với đo 2 chiều), nhiều cặp bị cổng loại. Những cặp vẫn lọt qua thì kéo track sai chỗ, tới lượt
   LiDAR sau residual LiDAR cũng lớn lên, và RMSE fused có thể còn tệ hơn chạy LiDAR một mình.
5. Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng?
   Giải thích vì sao lidar quyết định score/init/delete còn camera chỉ EKF update.
   Frame không có detection nào thì `meas_list` rỗng, không lấy được `meas.sensor`. Nhưng frame
   đó vẫn phải đi qua `manage_tracks` như một lượt LiDAR để các track trong FOV bị trừ điểm miss,
   không thì track ma không bao giờ bị xoá. Còn vì sao chỉ LiDAR được quyết định: LiDAR có
   detector thật và đo đủ 3D nên đủ tin để tạo track. Camera trong lab chỉ có tâm hộp 2D, không có
   độ sâu thì không khởi tạo được vị trí 3D, FOV lại hẹp, nếu cho camera trừ điểm thì xe ở ngoài
   ảnh bị xoá oan. Nên camera chỉ dùng để update `x` và `P`.
6. Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.
   Track mới có score 1/6, trạng thái `initialized`. Mỗi lượt LiDAR, ghép được thì +1/6 (tối đa
   1) và chuyển `tentative` nếu chưa confirmed; miss mà track nằm trong FOV thì −1/6. Score > 0.8
   (lớn hơn hẳn) thì thành `confirmed`. Confirmed rồi thì miss vài lần vẫn giữ confirmed, chỉ xoá
   khi score < 0.6. Track chưa confirmed thì xoá khi score ≤ 0. Ngoài ra nếu P[0,0] hoặc P[1,1]
   > 9 (lệch chuẩn hơn 3 m) thì xoá luôn, không cần xét score.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- Không

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (Claude Opus 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): sắp xếp thư mục dữ liệu và cài môi trường (CP0); implement Part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`); debug lỗi encoding cp1252 trên Windows và lỗi `float()` với ma trận 1×1; viết phần giải thích kết quả và 6 câu trả lời E–H.
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
