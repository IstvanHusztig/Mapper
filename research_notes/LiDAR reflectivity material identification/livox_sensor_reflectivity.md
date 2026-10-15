# Livox and low-cost 905 nm LiDAR reflectivity: definition, normalization, calibration practice

Research date: 2026-10-15. Note on access: livoxtech forum, docs.ouster.com, arxiv.org, ri.cmu.edu and several PDF hosts could not be fetched from this environment (DNS/proxy refusal), so many findings below are from search-result extracts of those pages, not full-text reads. Those are marked "(search extract)". GitHub raw files and local repo code were read directly.

Labels: **[OFFICIAL]** = manufacturer document/SDK; **[STAFF]** = manufacturer staff reply on a forum; **[COMMUNITY]** = users/third parties; **[PAPER]** = peer-reviewed or arXiv; **[CODE]** = read directly from source.

## 1. What does Livox "reflectivity" mean, and is it range/angle compensated?

### Takeaway
Livox reports an 8-bit "reflectivity" where 0–150 maps (nominally linearly) to 0–100 % Lambertian reflectance at 905 nm and 151–255 flags retroreflective targets. A Livox staff reply says the value is already compensated for distance (and other sensor factors) but NOT for incidence angle, i.e. it is approximately rho * cos(theta). Livox's own Mid-360 manual warns that below 2 m the reflectivity error is large and the value should only be used to tell retroreflective from diffuse. No public Livox document gives the accuracy, linearity, per-channel or temperature behaviour.

### Cited Findings
- [OFFICIAL] Livox Horizon manual: reflectivity is a number 0–255; 0–150 corresponds to 0–100 % reflectivity in the Lambertian reflection model; 151–255 corresponds to targets with retroreflective properties (search extract) — [Livox Horizon User Manual v1.0](https://www.livoxtech.com/3296f540ecf5458a8829e01cf429798e/assets/horizon/Livox%20Horizon%20user%20manual%20v1.0.pdf)
- [OFFICIAL] Mid-360 user manual repeats the 0–255 / 0–150 = 0–100 % Lambertian / 151–255 retroreflective definition, and warns: when the target is less than 2 m from the Mid-360 it "may result in a large reflectivity error", so the value should then only be used to distinguish retroreflective from diffuse targets (search extract) — [Livox Mid-360 User Manual v1.2 (2024.04)](https://www.sachtleben-technology.com/wp-content/uploads/sites/5/2024/07/LivoxMid-360UserManual.pdf); mirror: [aifitlab wiki copy](https://wiki.aifitlab.com/livox-lidar-docs/livox-mid-360-user-manual)
- [OFFICIAL] Mid-360: for objects 0.1–1 m away that are low-reflectivity or thin, "the detection effect cannot be guaranteed"; range spec 40 m @ 10 %, 70 m @ 80 % reflectivity; detection range shortens toward the upper edge of the vertical FOV (search extract) — [Mid-360 manual](https://www.sachtleben-technology.com/wp-content/uploads/sites/5/2024/07/LivoxMid-360UserManual.pdf), [Livox Mid-360 product page](https://www.livoxtech.com/mid-360)
- [OFFICIAL] Avia: cannot measure targets < 1 m, point cloud may be distorted at 1–3 m; effective range shrinks toward FOV edges; 190 m @ 10 %, 320 m @ 80 % (search extract) — [Avia specs](https://www.livoxtech.com/avia/specs), [Avia User Manual](https://www.sachtleben-technology.com/assets/downloads/livox-avia-user-manual.pdf)
- [STAFF] Livox forum: staff first wrote 0–155 for diffuse, then corrected to 0–150 after a user pointed out the overlap; the scale is Livox-specific; Lambertian vs retroreflective is distinguished by "optical analysis" (search extract) — [Livox forum thread 201](https://forum.livoxtech.com/thread-201-1-1.html)
- [COMMUNITY + STAFF] A user testing a retroreflective jacket saw values rising to ~250 only at about 17–20 m; the Livox reply said reflectivity measured at 20 m is "the most accurate" (implies the retro classification/value is range-dependent and least reliable at close range) (search extract) — [Livox forum thread 190](https://forum.livoxtech.com/thread-190-1-1.html)
- [STAFF] Livox Google-group reply: unlike conventional intensity, "Livox reflectivity has compensated all those factors (e.g., distance) but the incidence angle. Under a Lambertian assumption, the reflectivity is the object color (at 905 nm) times by the cosine value of the laser incidence angle." (search extract) — [livox-lidars Google group: "Reflectivity, minimum distance, and integration time settings"](https://groups.google.com/g/livox-lidars/c/4TphNgFpwZQ)
- [PAPER] Contrasting usage: Loam_livox (Mid-40) treats intensity as distance-dependent ("small intensity means the point is either far from the sensor or the object reflectivity is low") — [Lin & Zhang, Loam_livox, arXiv 1909.06700](https://arxiv.org/pdf/1909.06700). A Mid-40 evaluation reports intensity significantly influences range measurement at very close range (search extract) — [Initial investigation of a low-cost automotive lidar system (ResearchGate)](https://www.researchgate.net/publication/337644685_INITIAL_INVESTIGATION_OF_A_LOW-COST_AUTOMOTIVE_LIDAR_SYSTEM)
- [OFFICIAL / CODE] Livox-SDK2 point structs (`LivoxLidarCartesianHighRawPoint`, `...LowRawPoint`, `LivoxLidarSpherPoint`) carry `uint8_t reflectivity` and `uint8_t tag`, with no further documentation in the header — [Livox-SDK2 livox_lidar_def.h](https://raw.githubusercontent.com/Livox-SDK/Livox-SDK2/master/include/livox_lidar_def.h)
- [OFFICIAL] livox_ros_driver2 publishes reflectivity in the `intensity` field as 0.0–255.0 (PointXYZRTLT / PCL PointXYZI depending on `xfer_format`) (search extract) — [livox_ros_driver2 fork mirror](https://github.com/TixiaoShan/livox_ros_driver2)
- [OFFICIAL] Mid-360 `tag` byte (protocol doc): bits 0–1 = "glue"/adhesion points between adjacent objects, bits 2–3 = rain/fog/dust/fine particles, bits 4–5 = other low-reliability, bits 6–7 reserved; per 2-bit group 0 = high confidence, 1 = medium, 2 = low. Livox says points can be filtered on this (search extract; Livox's tag article uses a different bit-pair notation — verify against firmware) — [Livox Mid-360 communication protocol](https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html), [Livox Tag article](https://www.livoxtech.com/showcase/livox-tag)
- [OFFICIAL] Avia firmware v40.01.0506 improved point-cloud behaviour for retroreflective targets within 30 m (reported by a reseller blog) — [OpenELAB: Avia firmware update](https://openelab.io/blogs/learn/livox-avia-2-firmware-update-retroreflective-targets)

### Inferences
- If the staff statement holds, Livox reflectivity R ≈ k * rho * cos(theta) for diffuse targets: range normalization is (nominally) already done on-board, and the main remaining correction is incidence angle (divide by cos(theta) or a fitted f(theta)), plus a near-range correction below ~2 m, which matters a lot for caves where much of the wall is within 2 m.
- The 0–150 → 0–100 % mapping suggests rho_% ≈ R / 1.5 for diffuse targets, but nothing published confirms linearity; treat it as nominal and verify.
- Values ≥151 in a cave are likely wet/specular surfaces, survey reflector targets, or near-range artifacts; they should be excluded from diffuse-reflectance statistics.
- The tag bits (dust/mist confidence) are directly relevant to caves (breath vapour, mist, dust) and should be preserved per point.

### Gaps
- No official Livox document on reflectivity accuracy, linearity, temperature dependence, or per-channel (Mid-360 has multiple emitter/receiver channels with non-repetitive scanning) differences was found. GitHub issues in Livox-SDK2 / livox_ros_driver2 could not be searched (code search blocked).
- Saturation behaviour at very close range (0.1–0.4 m) is only implied by the "<2 m large error" and "0.1–1 m not guaranteed" statements; no measured curves found.
- Mid-40/70 and HAP specific reflectivity statements were not retrieved; it is assumed (not verified) they share the same 0–255 definition.

## 2. Ouster / Velodyne / Hesai equivalents and published calibration methods for 905 nm automotive LiDAR

### Takeaway
Ouster and Velodyne both provide a factory-calibrated, range-compensated 8-bit reflectivity separate from raw signal: 0–100 ≈ linear Lambertian %, 101–255 retroreflective (Ouster uses a log2 mapping above 100). Neither compensates incidence angle. The academic standard for uncalibrated multi-beam sensors is targetless, multi-view calibration of a per-beam response curve (Levinson & Thrun), and reference-surface methods for road markings (Jeong & Kim).

### Cited Findings
- [OFFICIAL] Ouster: calibrated reflectivity is an 8-bit packet field that scales measured signal photons by range and sensor sensitivity to estimate target reflectivity; factory-calibrated sensors are more accurate, uncalibrated ones use defaults; Rev 7 needs FW ≥ 3.0.0, Rev 8 FW ≥ 4.0; calibration V2 (FW ≥ 3.2) improves all ranges, most noticeably near range (search extract) — [Ouster docs: Calibrated Reflectivity](https://docs.ouster.com/sensor-docs/calibrated-reflectivity), [Ouster FW manual](https://ouster.com/sensor-docs/fw-user-manual/calibrated-reflectivity)
- [OFFICIAL] Ouster: values 0–100 map linearly to 0–100 % for Lambertian targets; 101–255 use a log2 mapping with linear interpolation for retroreflectors (search extract) — [Ouster docs: Calibrated Reflectivity](https://docs.ouster.com/sensor-docs/calibrated-reflectivity)
- [STAFF/COMMUNITY] Ouster forum: SIGNAL is the raw photon count; calibrated reflectivity can only be computed when range is known because the correction is largely range-dependent; since FW 3.1 reflectivity 0 always means range 0 (non-detection) (search extract) — [Ouster community: reflectivity 0](https://community.ouster.com/t/getting-many-points-with-reflectivity-as-0/401)
- [OFFICIAL via press] Velodyne "calibrated reflectivities": 0–100 for diffuse reflectors (approximation from the ratio of emitted to received power), 101–255 for retroreflectors from dirty/imperfect to ideal-angle — [Amerisurv, 2014](https://amerisurv.com/2014/03/19/velodyne-lidar-technology-achieves-calibrated-reflectivies-capability-enables-significant-cost-savings/); same split cited by [Siemens KB: physics-based lidar power calibration](https://support.sw.siemens.com/en-US/okba/KB000046045_EN_US/Physics-based-lidar-power-calibration/index.html)
- [PAPER] A VLP-16 study cites the manufacturer: intensity is factory-calibrated against a NIST-calibrated reflectance target and independent of distance (search extract; specific paper not identified in the result) — [Siemens KB / search context](https://support.sw.siemens.com/en-US/okba/KB000046045_EN_US/Physics-based-lidar-power-calibration/index.html)
- [PAPER] Levinson & Thrun, "Unsupervised Calibration for Multi-beam Lasers", ISER 2010; Levinson's thesis extends it to automatic intrinsic/extrinsic calibration from seconds of driving; approach assumes points lie on contiguous surfaces and different beams pass the same surface area during motion; intensity (remittance) is calibrated per beam (search extract) — [Levinson IV 2011 (CMU copy)](https://www.ri.cmu.edu/app/uploads/2017/12/levinson-iv2011.pdf), [Levinson thesis, Stanford](https://purl.stanford.edu/zx701jr9713)
- [COMMUNITY/STUDENT] CMU RISS poster: each beam has a nonlinear remittance error vs the world and vs other beams; solved with conditional expected value over beams plus modified EM — [Colby Horn poster](https://riss.ri.cmu.edu/education/colby-poster.pdf)
- [PATENT] Unsupervised multi-beam intensity calibration by aligning intensity distributions over a large area (hundreds of metres) where beams overlap, avoiding need for precise poses — [US 10107899](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10107899); [US 12135395 "Multi-beam lidar intensity calibration by interpolating learned response curve"](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12135395)
- [PAPER] Jeong & Kim, "LiDAR Intensity Calibration for Road Marking Extraction", UR 2018, pp. 455–460: intensity depends on reflectance, distance and incidence angle; uses each sensor's response relative to a reference surface (road) to estimate reflectance; validated by Otsu-threshold road-marking extraction — [KAIST repository](https://koasas.kaist.ac.kr/handle/10203/244114?mode=full)
- [PAPER] Kashani et al. 2015 review (Sensors 15(11) 28099–28128): standard terminology for intensity "normalization/correction/calibration"; factors = target properties + acquisition geometry (range, incidence angle) + instrument + atmosphere — [PMC4701271](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4701271/), [MDPI PDF](https://www.mdpi.com/1424-8220/15/11/28099/pdf)
- [PAPER] Radar-range-equation basis: backscatter intensity ∝ cos(incidence angle) (search extract) — [Remote Sensing 12, 2855 (2020)](https://doi.org/10.3390/RS12172855)
- [PAPER] Intensity-SLAM adds an intensity cost to geometric LiDAR SLAM and includes an intensity calibration step (details not retrieved) — [Wang et al., arXiv 2102.03798 (RA-L 2021)](https://arxiv.org/pdf/2102.03798); intensity-based place recognition — [arXiv 2103.09605](https://arxiv.org/pdf/2103.09605); InTEn-LOAM — [arXiv 2209.05708](https://arxiv.org/pdf/2209.05708)

### Inferences
- Even "calibrated" vendor reflectivity is only range-corrected; incidence-angle correction and near-range correction are always left to the user.
- Livox sits between Ouster/Velodyne (documented calibrated fields) and raw-intensity sensors: it ships a single "reflectivity" byte with a vendor claim of range compensation but no published calibration procedure.

### Gaps
- Hesai's reflectivity definition (scale, calibration) could not be found from this environment.
- Full text of Levinson & Thrun, Jeong & Kim, and Intensity-SLAM calibration models could not be retrieved; exact model forms (e.g. Jeong & Kim's angle model) are unverified.

## 3. How Mandeye / HDMapping / dMapper store intensity in LAZ

### Takeaway
Mandeye writes the Livox SDK2 reflectivity byte unchanged into the LAS `intensity` field (point format 1, 0.1 mm scale), the laser/LiDAR index into `user_data`, and the Livox `tag` into `classification`. dMapper reads `intensity` and `user_data` and writes intensity unchanged to its output LAZ; it does not carry the tag through. No intensity-normalization tool was found in the Mandeye/HDMapping ecosystem.

### Cited Findings
- [CODE] Mandeye `LivoxClient.cpp`: stores only `kLivoxLidarCartesianCoordinateHighData` packets, copies the raw SDK point struct (`point.point = p_point_data[i]`), sets `point.laser_id = handleToLidarId(handle)` (index of the LiDAR serial number, 255 if unknown, i.e. a per-device id, not a per-emitter channel) — [mandeye_controller LivoxClient.cpp](https://raw.githubusercontent.com/JanuszBedkowski/mandeye_controller/main/code/LivoxClient.cpp)
- [CODE] Mandeye `save_laz.cpp`: `header->point_data_format = 1`; scale 0.0001 (0.1 mm); `point->intensity = p.intensity` (no scaling); `point->user_data = p.laser_id`; `point->classification = p.tag`; `point->gps_time = p.timestamp * 1e-9` — [mandeye_controller save_laz.cpp](https://raw.githubusercontent.com/JanuszBedkowski/mandeye_controller/main/code/save_laz.cpp)
- [CODE] dMapper (this repo) reads `point->intensity` into `Point3Di::intensity`, `user_data` into `lidarid`, `gps_time` into timestamp: `/home/user/Mapper/src/infrastructure/recording_folder_source.cpp` lines 189–206; output LAZ uses point format 1, 0.0001 scale and writes `intensity` unchanged (`src/infrastructure/laz_export.cpp` l.79–127, `src/infrastructure/folder_result_sink.cpp` l.43–60). `classification` (Livox tag) is not read or written.
- [CODE] dMapper's loader comment refers to a "Unitree L2 IMU lever arm" (`recording_folder_source.cpp` l.192–194), so the user's recordings may come from a Unitree L2 rather than a Livox; its intensity semantics would then differ from Livox's.

### Inferences
- Because LAS intensity is uint16 and Mandeye writes 0–255, a normalized value could be stored in the same field with a documented scale (e.g. reflectance % × 100) or in an extra-bytes field, keeping the raw value.
- Preserving the tag (classification) and the per-point range/incidence angle (or sensor position via trajectory) in the output would be needed for any later normalization.

### Gaps
- HDMapping's own LAZ export file could not be located (guessed path 404; GitHub code search blocked). Whether HDMapping has any intensity colouring or normalization tool is unverified.
- No Mandeye/HDMapping publication on intensity was found.
- What the Unitree L2 reports as intensity was not researched.

## 4. Studies using Livox / low-cost 905 nm sensors for material, wetness, mining, caves

### Takeaway
No study was found that uses Livox reflectivity for material classification, wetness or cave surveys. General TLS literature shows wetness strongly reduces intensity (30–50 % for brick), and that intensity separates materials only when range and incidence angle vary little.

### Cited Findings
- [PAPER] Moisture can cause a 30–50 % drop in reflectance of brick samples (Kaasalainen et al., cited) — [PMC9402562: 3D laser scanning to monitor moisture content](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9402562/)
- [PAPER] Intensity cannot yet be a primary classifier in complex natural scenes because of range, incidence angle, wet/dry state and lack of cross-manufacturer standards; in simple cases with little change in distance and angle, it separates materials fairly well — [Brodu & Lague, arXiv 1107.0550](https://arxiv.org/pdf/1107.0550)
- [PAPER] Livox Horizon used for underground metal-mine mapping (geometry only, no reflectivity analysis) — [Measurement, 2023 (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0263224123011454)
- [PAPER] Full-waveform flash lidar for material segmentation notes prior lidar material classification relied mainly on a single intensity value — [arXiv 2305.04334](https://arxiv.org/pdf/2305.04334)
- [PAPER] Radiometric fingerprinting of object surfaces using mobile laser scanning and semantic 3D road models (2026; content not retrieved) — [arXiv 2603.11252](https://arxiv.org/pdf/2603.11252)
- [PAPER] "Super LiDAR Intensity for Robotic Perception" (2025; content not retrieved) — [arXiv 2508.10398](https://arxiv.org/pdf/2508.10398)
- [PAPER] Fog effect on ToF LiDAR (reduced returns/intensity) — [arXiv 2003.06660](https://arxiv.org/pdf/2003.06660)

### Inferences
- In caves, wet vs dry rock will likely dominate intensity differences as much as lithology does; a "wetness" interpretation needs angle/range-normalized values first.

### Gaps
- No Livox-specific material, wetness, mining-reflectivity or cave study found; road-marking work with Livox not found.

## 5. Self-calibration from SLAM data (multi-view consistency, no targets)

### Takeaway
The established targetless approach is: register the scans (SLAM), find many observations of the same surface patch from different ranges/angles/beams, and fit correction curves so all observations of a patch agree (Levinson & Thrun per-beam EM; distribution alignment patents; reference-surface method of Jeong & Kim). No paper was found applying this specifically to Livox, but the method transfers directly to handheld SLAM data.

### Cited Findings
- [PAPER] Levinson & Thrun: unsupervised calibration from arbitrary, unknown environments assuming different beams observe the same contiguous surfaces during motion; per-beam remittance response estimated with EM (search extract) — [Levinson thesis](https://purl.stanford.edu/zx701jr9713), [IV 2011](https://www.ri.cmu.edu/app/uploads/2017/12/levinson-iv2011.pdf)
- [PATENT] Aligning intensity distributions across beams over a large shared area, without precise poses — [US 10107899](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10107899), [US 11029399](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11029399)
- [PATENT] Simultaneous online LiDAR intensity calibration and road-marking change detection — [US 12190607](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12190607)
- [PAPER] Mild viewpoint changes give mild intensity variation of the same 3D point (the basis for multi-view consistency) — [arXiv 2103.09605](https://arxiv.org/pdf/2103.09605)
- [PAPER] Analogous self-supervised approach for range bias using map consistency across viewpoints — [arXiv 2303.01123](https://arxiv.org/html/2303.01123v4)

### Inferences
- A practical model for dMapper: for each map voxel/patch p with estimated normal, each observation i gives R_i = g(r_i) · f(theta_i) · c_ch · rho_p (+ noise); solve jointly (alternating least squares in log space) for rho_p, a range curve g (expected near-flat above ~2 m if Livox's claim holds, important below 2 m), an angle curve f (start from cos theta), and optional per-device/channel gains c_ch. Exclude R ≥ 151, low-confidence tag points, and grazing angles (> ~70°).
- dMapper already has the inputs: optimized poses give sensor origin per point (range, incidence angle from NDT cell normals), and caves give dense revisits of the same wall from many ranges.

### Gaps
- No published Livox-specific self-calibration; the actual near-range curve and angle curve for Mid-360 must be measured on the user's data.
