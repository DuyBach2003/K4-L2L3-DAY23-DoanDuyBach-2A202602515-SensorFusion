# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Đoàn Duy Bách
- MSSV: 2A202602515
- Email: bachtipch@gmail.com
- Link repo (fork): https://github.com/DuyBach2003/K4-L2L3-DAY23-DoanDuyBach-2A202602515-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): ghi trên LMS (commit cuối của `main` sau khi push); artifacts chấm điểm được sinh ở commit `f130e5e` (CP5), code E–H không đổi sau lần chạy đó

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: 0.9700934579439252, 0.7004048582995951, 519 / 16 / 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.15032267351421266 m, 502, 11.343646898575209 m², 0, 239, 2.522613065326633
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: 0.1358667766766506 m, 502, 9.266810064260428 m², 0, 239, 2.522613065326633
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: xem các ý dưới đây.

**Hai RMSE so sánh được trực tiếp vì số đếm giống hệt nhau.** Cả hai mode có
`matches = 502`, `ghost_track_frames = 0`, `missed_gt_frames = 239`,
`mean_confirmed_tracks = 2.5226`. Trong `grade_run.log`, `confirmed` và `matches` của
`lidar` và `fused` trùng nhau ở **199/199 frame**, các trường `det_*`, `valid_gt` cũng
trùng ở mọi frame. Camera không tạo, xác nhận hay xoá track nào; nó chỉ đổi trạng thái
EKF. Vì vậy hai RMSE được tính trên đúng cùng 502 cặp track–GT.

| | RMSE (m) | `sum_sq_err` (m²) | matches | ghost | miss | `precision_track` | `coverage` |
|---|---|---|---|---|---|---|---|
| lidar | 0.1503 | 11.3436 | 502 | 0 | 239 | 502/502 = 1.000 | 502/519 = 0.967 |
| fused | 0.1359 | 9.2668 | 502 | 0 | 239 | 502/502 = 1.000 | 502/519 = 0.967 |

- **Fused tốt hơn LiDAR trên toàn segment:** RMSE giảm 0.0145 m (−9.6%), `sum_sq_err`
  giảm 18.3%. `sum_sq_err` của hai mode khác nhau ở 195/199 frame, fused thấp hơn ở
  137 frame. `rmse_fused − rmse_lidar = −0.0145 m ≤ 0.05 m`. Cả hai mode đều có
  RMSE ≤ 0.45 m, `precision_track` ≥ 0.75 và `coverage` ≥ 0.70.
- **Ghost = 0:** detector có 16 FP, nhưng track mới cần ít nhất 5 frame có hit lidar
  (nhanh nhất là 5 frame liên tiếp, score 1/6 → 5/6 > 0.8) mới được confirmed, nên FP
  rời rạc bị xoá trước khi thành confirmed track.
- **Miss = 239:** 222 trong số đó là xe detector bỏ sót (`det_fn = 222`, recall 0.70),
  và `coverage` không phạt các xe này. Phần còn lại đến từ độ trễ xác nhận. Ví dụ ở
  frame 0–3, 2 xe được phát hiện nhưng `confirmed = 0`, `misses = 2`; tới frame 4 cả hai
  mode mới có `confirmed = 2` (dòng 4–5 và 203–204 của `grade_run.log`). Cộng theo
  frame, `max(0, misses − det_fn)` bằng 22 track-frame bị trễ. Ngược lại, có ít nhất
  5 GT-frame detector bỏ sót nhưng track vẫn ghép được nhờ bước predict
  (239 − 222 = 22 − 5).
- **Theo trục** (tính lại bằng [`bonus/calibration_sweep.py`](bonus/calibration_sweep.py);
  script tái tạo đúng RMSE của cả hai mode): lidar x/y/z = 0.108/0.079/0.068 m, fused
  = 0.093/0.062/0.077 m. Camera đo hướng nhìn rất chính xác: σ = 5 px với
  f = 2083 px là khoảng 2.4 mrad, tức khoảng 0.05 m vuông góc tia nhìn ở 20 m. Nhờ đó
  sai số ngang y giảm, và x cũng giảm nhẹ.
  - Ngược lại, z tệ hơn vì đo camera có **bias**: trong 509 lần update camera,
    innovation trung bình là γ_u = −11.9 px, γ_v = +7.6 px, trong khi một phép đo
    không chệch phải có trung bình ≈ 0.
  - Tâm hộp 2D của nhãn không trùng hình chiếu tâm hộp 3D. Nhiều khả năng đây là do
    phối cảnh: camera đặt ở độ cao 2.12 m, cao hơn tâm xe (`center_z` của GT có trung
    vị 0.94 m), và mặt gần của xe chiếm phần lớn hộp 2D.
  - Bias này tạo ra sự "giằng co" đo được giữa hai cảm biến: residual ngang của lidar
    trên track confirmed có trung bình −0.083 m ở mode fused, so với −0.002 m ở
    lidar-only.
- **Theo thời gian** (tính từ `grade_run.log`): ở frame 0–15, fused tệ hơn (0.161 so
  với 0.120 m); ở frame 16–49 hai mode gần bằng nhau (+0.006 m); từ frame 50 fused tốt
  hơn rõ (−0.029, −0.020, −0.015 m cho các khoảng 50–99, 100–149, 150–198). Chỉ nhìn
  RMSE tổng sẽ không thấy giai đoạn đầu camera làm hại.
- **Giới hạn:** đo camera ở lab là tâm hộp 2D **ground-truth** của camera FRONT, cộng
  nhiễu Gauss 0.5 px theo seed (`run_lab.py` dòng 97), không phải output của một
  detector ảnh; còn `R` giả định σ = 5 px. Vì vậy mức cải thiện của fused **không**
  chứng minh một camera detector thật sẽ giúp được như vậy. Ngược lại, bias tâm 2D–3D ở
  trên cho thấy ngay cả nhãn GT cũng không phải phép đo không chệch của tâm 3D.

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

1. **Khác biệt đo lidar 3D và camera 2D trong EKF (`z`, `R`)?**
   - **Lidar:** `z = (x, y, z)ᵀ` là tâm hộp 3D (m), 3×1, với
     `R = diag(0.1², 0.1², 0.1²)` m². `h(x)` là vị trí track trong hệ lidar, tức một phép
     biến đổi tuyến tính, nên `H = [I₃ | 0]` (3×6) và bước update EKF trùng với KF
     tuyến tính.
   - **Camera:** `z = (u, v)ᵀ` là pixel, 2×1, với `R = diag(5², 5²)` px²
     ([`build_camera_measurement`](workspace/camera_fusion.py#L82-L95)). `h(x)` là phép
     chiếu pinhole phi tuyến `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s`
     ([`camera_measurement_prediction`](workspace/camera_fusion.py#L55-L79)), nên EKF
     tuyến tính hoá bằng Jacobian `H` 2×6 (platform tính, cột vận tốc bằng 0) tại
     trạng thái dự báo. Camera không đo độ sâu: nó chỉ ràng buộc hướng nhìn, và sai số
     5 px quy ra mét tăng theo khoảng cách (≈ 5·d/f).
   - Cả hai đi qua cùng [`ekf_update`](workspace/kalman.py#L111-L130); chỉ khác `meas.z`,
     `meas.R`, `get_hx`, `get_H`. Số chiều đo cũng đổi bậc tự do của cổng χ²
     (3 → 12.84; 2 → 10.60).
2. **Vì sao cần gating Mahalanobis trước khi gán?**
   - Greedy luôn chọn cặp rẻ nhất. Nếu không có cổng, một FP, một xe khác hoặc nhãn
     camera của xe ở xa vẫn bị gán và kéo trạng thái lệch: track "nhảy" sang xe khác,
     sinh ghost hoặc đổi ID.
   - [`chi2_gate`](workspace/association.py#L41-L53) loại cặp có
     `d² = γᵀS⁻¹γ > χ²₀.₉₉₅(dim)`. Trong
     [`association_cost_matrix`](workspace/association.py#L56-L78), cặp bị loại hoặc
     ngoài FOV có chi phí `inf` và không bao giờ được ghép: đo lidar đó sẽ thành track
     mới, còn track bị tính miss.
   - Mahalanobis chuẩn hoá innovation theo `S = HPHᵀ + R`, nên cổng **tự co giãn theo
     độ bất định**. Ở lần ghép đầu của track mới (σ_v = 50 m/s), `P_xx` sau predict
     = 0.01 + 0.1²·2500 + 0.3 ≈ 25.3 m², nên bán kính cổng ≈ √(12.84·25.3) ≈ 18 m.
     Với track confirmed ổn định, `S_xx` trung vị ≈ 0.35 m² nên bán kính chỉ ≈ 2.1 m;
     nó không thể nhỏ hơn ≈ 2 m vì `P_xx` sau predict luôn ≥ `Q` = 0.3 m².
   - Euclidean dùng ngưỡng mét cố định: hoặc quá chặt với track mới, hoặc quá lỏng với
     track ổn định. Nó cũng bỏ qua hướng của bất định và không so được pixel của camera.
   - Bằng chứng: khi camera lệch yaw 1°, cổng chặn 96% đo camera (509 → 20 lần update;
     xem Bonus).
3. **Pipeline là track-then-fuse hay fuse-then-track? Chỉ ra trên log `fusion-run-lab`.**
   - **Track-then-fuse.** Có một danh sách track duy nhất. Mỗi frame, mọi track được
     `KF.predict` đúng một lần, sau đó AssocL `associate_and_update(..., lidar_sensor)`,
     rồi AssocC với `camera_sensor` nếu frame có nhóm nhãn FRONT
     ([`run_lab.py` dòng 199–209](../platform/fusion_lab/scripts/run_lab.py#L199-L209)).
     Không có bước gộp point cloud với ảnh trước detection: detector chỉ thấy BEV lidar.
   - **Trên log:**
     - Detection chạy trước và độc lập với fusion: `det_tp/det_fp/det_fn/valid_gt` của
       `mode=lidar` và `mode=fused` giống nhau ở mọi frame.
     - Lượt camera chỉ tinh chỉnh trạng thái: `confirmed` và `matches` trùng ở 199/199
       frame, còn `sum_sq_err` khác ở 195/199 frame.
     - Ví dụ dòng 61 và 260 (frame 60): cả hai mode có `det_tp = 3`, `confirmed = 2`,
       `matches = 2`, `misses = 1`, nhưng `sum_sq_err` là 0.0308 m² (lidar) và
       0.0346 m² (fused).
4. **Nếu camera lệch calibration, triệu chứng gì trên innovation/residual?**
   - Innovation camera không còn trung bình gần 0 mà lệch có hệ thống khoảng
     −f·tan(δ). Với yaw δ = 0.1°/0.25°/0.5°, `mean γ_u` đi từ −11.9 px lần lượt xuống
     −15.6/−20.5/−27.6 px.
   - NIS trung bình tăng 2.25 → 3.09 → 4.71 → 7.64, trong khi giá trị kỳ vọng là 2 và
     cổng ở 10.60. Bộ lọc trở nên "không nhất quán": innovation lớn hơn mức `S` dự
     đoán.
   - Tỷ lệ đo camera qua cổng giảm (509 → 506 → 484 → 304 lần update) và RMSE ngang tăng
     (`rmse_y` 0.062 → 0.172 m).
   - Lệch nhỏ (≤ 0.5°) **lọt cổng** và kéo track lệch ngang: đây là vùng nguy hiểm nhất.
     Lệch lớn (≥ 1° ≈ 36 px) bị cổng chặn gần hết nên fused lùi về gần lidar-only.
   - Residual lidar xuất hiện thành phần kéo ngược lại (hai cảm biến "giằng co"):
     γ_y trung bình của lidar trên track confirmed là −0.002 m khi tắt camera, −0.083 m
     ở 0°, −0.115/−0.150 m ở 0.1°/0.25°, và về ≈ 0 khi camera bị chặn (≥ 1°). Bảng số
     liệu ở mục Bonus.
5. **Vì sao `associate_and_update(..., sensor)` cần sensor tường minh ở frame rỗng? Vì
   sao lidar quyết định score/init/delete còn camera chỉ EKF update?**
   - Khi `meas_list` rỗng thì không có `meas.sensor` để biết đây là lượt nào, nhưng hai
     trường hợp phải xử lý khác nhau. Lượt lidar rỗng vẫn phải trừ score các track trong
     FOV lidar và xoá track hết score. Lượt camera rỗng thì không được đụng tới vòng
     đời.
   - [`associate_and_update`](workspace/association.py#L112-L142) luôn kết thúc bằng
     `manager.manage_tracks(unassigned_tracks, unassigned_meas, sensor)`, và manager rẽ
     nhánh theo `sensor.name`
     ([`manager.py` dòng 95, 111](../platform/fusion_lab/tracking/manager.py#L88-L125)).
   - Nếu suy sensor từ danh sách đo hoặc dùng mặc định, một frame lidar không có
     detection sẽ bỏ qua miss, nên ghost sống mãi. Ngược lại, một frame camera rỗng bị
     coi là lidar sẽ trừ điểm hoặc xoá track sai, tức hai lần miss mỗi frame.
   - Các test `test_empty_lidar_frame_scores_then_deletes_exhausted_track` và
     `test_camera_pass_never_deletes_or_spawns` kiểm tra đúng hai trường hợp này.
   - **Lidar quyết định tồn tại** vì detector lidar phủ toàn cửa sổ BEV 0–50 m × ±25 m
     và cho vị trí 3D đầy đủ, nên khởi tạo được `x` và là một nguồn bằng chứng nhất quán
     mỗi frame.
   - Camera chỉ có hướng 2D (không có độ sâu nên không khởi tạo được track 3D), và FOV
     hẹp (−24.8°…+24.7° với camera FRONT của segment này). "Miss camera" vì thế không
     có nghĩa là xe biến mất.
   - Nếu camera cũng cộng điểm, mỗi frame có 2 lần cộng: xác nhận nhanh gấp đôi, ghost
     khó bị xoá, và score mất ý nghĩa "tỷ lệ hit trong `window` frame lidar". Hơn nữa, đo
     camera ở lab lấy từ nhãn GT, nên cho nó đổi vòng đời là rò rỉ GT vào quyết định tồn
     tại.
   - Log xác nhận: `confirmed` trùng nhau ở 199/199 frame giữa hai mode.
6. **Nêu điều kiện xác nhận, giữ confirmed sau miss, và điều kiện xóa track.**
   - **Khởi tạo** ([`init_track_state_from_meas`](workspace/track_management.py#L18-L38)):
     từ một đo lidar chưa được ghép, `x = [p_veh; 0; 0; 0]`, `P_pos = M R Mᵀ`,
     `P_vel = diag(50², 50², 5²)`, `score = 1/6`, `state = "initialized"`.
   - **Xác nhận** ([`update_track_score`](workspace/track_management.py#L41-L64)): mỗi
     lần hit lidar, `score = min(1, score + 1/6)`; track chưa confirmed thành
     `confirmed` khi `score > 0.8`, ngược lại là `tentative`. Do đó nhanh nhất cần
     5 frame có hit liên tiếp, tính cả frame khởi tạo (1/6 → … → 5/6 ≈ 0.833 > 0.8),
     khớp với log: 2 xe xuất hiện từ frame 0 được confirmed ở frame 4.
   - **Giữ confirmed sau miss:** miss chỉ được tính khi track nằm trong FOV lidar
     (`manage_tracks`). Khi đó `score −= 1/6` nhưng `state` vẫn là `confirmed`, không bị
     hạ về `tentative`.
   - **Xoá** ([`should_delete_track`](workspace/track_management.py#L67-L86), các điều
     kiện nối bằng OR):
     - `P[0,0]` hoặc `P[1,1]` > `max_P` = 9 m² (σ > 3 m), bất kể score;
     - track confirmed có `score < 0.6`;
     - track chưa confirmed có `score ≤ 0`.
   - Hệ quả: track confirmed đang có score 1.0 chịu được 2 miss liên tiếp (5/6 và 4/6 đều
     ≥ 0.6) và bị xoá ở miss thứ 3 (3/6 = 0.5). Track vừa khởi tạo (1/6) bị xoá ngay ở
     miss đầu tiên.
   - Lượt camera không bao giờ gọi các hàm này.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- **Phân tích calibration (+4): có.**
  - Script [`bonus/calibration_sweep.py`](bonus/calibration_sweep.py) chạy đúng vòng
    lặp của `--fusion fused` với workspace của bài: một predict, lượt lidar, rồi lượt
    camera. Khác biệt duy nhất là pose camera FRONT bị xoay thêm một góc yaw δ quanh
    trục z của xe trước khi tracking.
  - Detection và pixel camera có nhiễu seed 0 được tính một lần rồi dùng chung cho mọi
    mức lệch. Mức δ = 0 tái tạo đúng `tracking.fused` của lần chấm (RMSE 0.13587 m), và
    lần chạy tắt camera tái tạo đúng `tracking.lidar` (0.15032 m).
  - Số liệu nằm trong [`bonus/calibration_sweep.json`](bonus/calibration_sweep.json).
  - Camera FRONT: f = 2083.1 px, nên 1° yaw ≈ 36.4 px theo phương u.

  | Lệch yaw δ | ≈ px | RMSE (m) | `rmse_y` (m) | Update camera qua cổng (trên 2799 đo) | mean γ_u (px) | mean NIS | γ_y lidar TB (m) |
  |---|---|---|---|---|---|---|---|
  | tắt camera (lidar) | – | 0.1503 | 0.0790 | 0 | – | – | −0.002 |
  | 0° | 0 | 0.1359 | 0.0623 | 509 | −11.9 | 2.25 | −0.083 |
  | 0.1° | 3.6 | 0.1458 | 0.0844 | 506 | −15.6 | 3.09 | −0.115 |
  | 0.25° | 9.1 | 0.1755 | 0.1295 | 484 | −20.5 | 4.71 | −0.150 |
  | 0.5° | 18.2 | 0.2129 | 0.1718 | 304 | −27.6 | 7.64 | −0.129 |
  | 1° | 36.4 | 0.1738 | 0.1140 | 20 | −73.5 | 3.43 | −0.004 |
  | 2° | 72.7 | 0.1780 | 0.1038 | 19 | −80.6 | 3.85 | −0.006 |

  Ở mọi mức lệch, `matches/ghost/miss` = 502/0/239: lệch calibration không đổi vòng đời
  track, vì camera không bỏ phiếu cho sự tồn tại.

  Nhận xét:
  - Innovation lệch dần đúng chiều và gần đúng độ lớn dự đoán: Δ`mean γ_u` = −3.7 px ở
    0.1° (dự đoán −3.6) và −8.6 px ở 0.25° (dự đoán −9.1). Ở 0.5° chỉ còn −15.7 px so
    với dự đoán −18.2, vì các innovation lớn nhất đã bị cổng loại (chọn mẫu).
  - Cổng **không chặn** khi độ lệch nhỏ so với √S_uu. Ở 0°, `S_uu` của các update camera
    nằm trong khoảng 60–184 px² (p10–p90), tức σ ≈ 8–14 px, nên lệch ≤ 0.25° (≤ 9 px)
    vẫn qua cổng và kéo track lệch ngang: `rmse_y` tăng gấp đôi, và lidar phải kéo
    ngược lại mạnh hơn (γ_y lidar −0.083 → −0.150 m).
  - Ở 0.5°, cổng bắt đầu loại khoảng 40% đo. Ở ≥ 1°, NIS của hầu hết đo vượt 10.60 nên
    96% bị chặn, và fused lùi về gần lidar-only.
  - RMSE ở 1° và 2° vẫn cao hơn lidar-only (0.174–0.178 so với 0.150 m) vì 19–20 lần
    update còn lọt cổng. Đó không phải xe ở gần (độ sâu 10.8–49.8 m) mà là các track
    còn bất định cao: `S_uu` trung vị ≈ 695 px² so với 89 px² ở 0°, nên cổng rất rộng.
    Innovation trung vị chỉ −19 px (1°) và −9 px (2°), nhưng trung bình lên tới
    −73.5/−80.6 px do vài giá trị rất lớn.
  - Kết luận: lệch calibration nhỏ nguy hiểm hơn lệch lớn, vì gating chỉ chặn được sai
    lệch lớn hơn độ bất định mà bộ lọc tự khai báo.
- Export track sang CVAT (+3): Không.
- Trực quan hoá track/đo trên BEV hoặc ảnh (+3): Không.

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (Anthropic, model Claude Opus 5.5) trong VS Code
- Dùng cho phần nào (hàm, câu hỏi, debug): Claude Code thực hiện phần lớn bài: dựng môi trường và tải dữ liệu; viết toàn bộ các hàm Part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`); chạy `pytest` và lần chạy chấm điểm `fusion-run-lab --fusion compare --seed 0`; viết script bonus `bonus/calibration_sweep.py`; soạn nháp các câu trả lời, bảng số liệu và nhận xét trong file này
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` cho 128 passed, không còn `failed`/`xfailed`; công thức được đối chiếu với docstring, các gợi ý TODO và `docs/HUONG_DAN_KY_THUAT.md`; `metrics.json` được kiểm tra khớp `grade_run.log` bằng `validate_metrics_records` của platform và `tools/check_submission.py`; mọi số liệu trong báo cáo lấy từ `student/artifacts/metrics.json`, `grade_run.log` và `student/bonus/calibration_sweep.json` (script bonus tái tạo đúng RMSE hai mode của lần chạy chấm); artifacts không bị sửa tay

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
