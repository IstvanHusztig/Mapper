# Accuracy specification, measurement and acceptance for cave surveys and handheld SLAM LiDAR maps

Research status: about 20 tool calls. Many primary documents could not be fetched (UIS v2 PDF 404, Survex list 403, Acta Montanistica host unreachable, vendor PDFs binary). Items not verified from a primary page are marked as such. Vendor claims and independent measurements are kept apart.

## 1. Cave survey grading systems (BCRA, UIS) and loop misclosure in practice

### Takeaway
BCRA grades give instrument and station-position tolerances only (Grade 5: +-1 deg, station <10 cm; Grade 6: +-0.5 deg, station <2.5 cm). The UIS ontology page (a v1 table) adds an "expected error ratio" per grade (G3 <10 %, G4 <5 %, G5 <2 %, G6 <1 %), and letter suffixes (A-F; B = loops closed and adjusted, E = electronic download). Practical loop misclosure of good compass/tape surveys is about 1-2 %; DistoX users use absolute thresholds (0.5 m) and report 0.1 % on well-calibrated loops.

### Cited Findings
- BCRA Grade 3: rough magnetic survey, horizontal and vertical angles +-2.5 deg, distances +-50 cm, station position error <50 cm. — [BCRA surveying](https://bcra.org.uk/surveying)
- BCRA Grade 5: angles +-1 deg, distances recorded to nearest cm, station positions identified to <10 cm (measurements from within a 10 cm sphere around the station; coordinates calculated, not hand-plotted). — [BCRA surveying](https://bcra.org.uk/surveying), cross-checked via search snippet of the same page
- BCRA Grade 6: compass and clinometer +-0.5 deg, station position error <+-2.5 cm (needs tripods or fixed markers such as roofhooks). — [BCRA surveying](https://bcra.org.uk/surveying)
- BCRA Grade X: based mainly on theodolite or total station; Grades 2 and 4 are intermediates "use only if necessary" (when conditions prevented the next higher grade). — [BCRA surveying](https://bcra.org.uk/surveying)
- BCRA detail classes: A = from memory; B = estimated and recorded in cave; C = measured at stations only; D = measured at stations and wherever needed for significant changes in dimensions; X-grade pairs with XA-XD. — [BCRA surveying](https://bcra.org.uk/surveying)
- UIS grades run -1 (no survey, database only), 0 (ungraded), 1 (sketch from memory), ... 6, and X (theodolite or comparable). UIS v2 PDF is dated 14 Sep 2012 (Survey and Mapping Working Group); records are written as e.g. "UISv1 Grade: 6-4-BEF" (survey grade - map detail grade - suffix letters). — [UIS ontology mapping grades](https://ontology.uis-speleo.org/mappingGrades/); search listing of [UIS Informatics Commission](https://uisic.uis-speleo.org/wgsurmap.html)
- UIS ontology page (V1 only, no v2 on that page) numbers: G3 length precision 0.5 m, compass 5 deg, expected error ratio <10 %; G4 length 0.1 m, compass/clino 2 deg, <5 %; G5 length 0.05 m, 1 deg, <2 %; G6 length 0.02 m, 0.25 deg, <1 %. — [UIS ontology mapping grades](https://ontology.uis-speleo.org/mappingGrades/). Note: differs from BCRA (G5 distance "nearest cm", G6 +-0.5 deg), so the UIS table is not a copy of BCRA's.
- UIS suffixes: A nothing done to gain extra certainty; B survey loops closed and adjusted; C instruments and people checked/corrected for anomalies; D checked by electromagnetic methods; E data downloaded electronically, not transcribed manually; F entrances precisely measured. None are numeric. — [UIS ontology mapping grades](https://ontology.uis-speleo.org/mappingGrades/)
- A UIS grades document states calibrated DistoX, DUSI or comparable instruments attain grade 5, and grade 6 if the length measurement has been calibrated (secondary snippet, not read in full). — [studylib UIS Mapping Grades](https://studylib.es/doc/9178138/uis-mapping-grades)
- Loop quality as percent error = closure error / loop length (example 15 ft on 1873 ft = 0.98 %); Compass docs caution that this ignores the claimed instrument accuracy and that longer loops should have smaller percentages, so no fixed threshold exists. — [Compass loop quality](https://fountainware.com/compass/HTML_Help/Project_Manager/loopquality.htm)
- Survex list user: with DistoX they check for errors if a loop closure exceeds 0.5 m regardless of length; with compass/clino/tape they were happy with <2 %. — [Survex list, Loop Closures](https://lists.survex.com/pipermail/survex/2014-November/001842.html) (from search summary; page not read in full)
- Caver report: 2300 ft loop closed at 0.1 % after DistoX calibration (Warm River Cave). — [texascavers](https://www.mail-archive.com/texascavers@texascavers.com/msg73412.html) (snippet-level)
- Cavway X1 (DistoX successor, vendor-side article): 0.1 % over 470 m loop. — [Scintilena](https://www.scintilena.com/cavway-x1-just-months-after-production-begins-2000-kilometers-and-zero-error-the-new-frontier-of-cave-surveying-instruments/01/01/) (marketing, treat as vendor claim)

### Inferences
- A dMapper-style acceptance number: for a Grade-5 equivalent claim expect <=2 % loop misclosure, Grade 6 equivalent <=1 %; DistoX-class practice suggests 0.1-0.5 % is achievable on good loops. For a 100 m traverse that is 0.1-1 m allowed misclosure (1 m at 1 %), versus 2 m at the 2 % level.
- Station position error (2.5-10 cm) is a per-station tolerance, not a drift budget; map-to-map comparisons with LiDAR cannot reach "Grade 6" station placement but can be compared against the closure percentage.

### Gaps
- Exact UISv2 numbers and whether v2 changed the error ratios: the PDF (karstworlds.com/UISmappingGrades.pdf) returned 404; not verified.
- Peer-reviewed (CREG, Int. J. Speleology) DistoX misclosure statistics not found; only forum-level reports.

## 2. Commercial handheld SLAM: stated accuracy, independent evaluations, field guidance

### Takeaway
Vendors state relative/absolute accuracies of 5-30 mm (1 sigma, in controlled environments); independent underground tests show centimetre-to-decimetre global errors that grow with trajectory length, mostly along the tunnel axis, and the vendors and authors all recommend frequent loop closure or control points.

### Cited Findings (vendor claims)
- Emesent Hovermap ST: SLAM drift +-0.03 %, LiDAR accuracy +-30 mm, mapping accuracy +-20 mm general, +-15 mm typical underground/indoor, +-5 mm isolated change detection. ST-X: +-15 mm general, +-10 mm typical indoor/underground, +-5 mm change detection; drift not specified. Page flagged stale. — [Emesent Hovermap specifications](https://knowledge.emesent.com/articles/hovermap-specifications). Implied: 0.03 % = 3 cm per 100 m (my arithmetic).
- Emesent KB notes featureless tunnels make drift removal harder and recommends control points along the full tunnel length (via search summary). — [Emesent scanning roads and tunnels](https://knowledge.emesent.com/articles/scanning-roads-and-tunnels)
- NavVis VLX 3: point cloud accuracy 5 mm (1 sigma) in a dedicated 500 m2 test environment; absolute accuracy depends on environment size and is controlled via control points; VLX 2 listed at 6 mm. — [NavVis device comparison matrix](https://knowledge.navvis.com/docs/device-comparison-matrix); [VLX 3 spec](https://www.navvis.com/resources/specifications/navvis-vlx-3)
- NavVis scanning tips: loop closures every 15-30 m and walk slowly (search summary). — [NavVis scanning tips](https://knowledge.navvis.com/docs/scanning-tips-1)
- GeoSLAM ZEB Horizon: up to 6 mm relative accuracy (processed in GeoSLAM Connect); another source quotes 1-3 cm accuracy depending on environment, used in Lombrives cave with known control points. — [geo-matching ZEB Horizon](https://geo-matching.com/products/zeb-horizon); [GIM International, mapping one of Europe's largest caves](https://www.gim-international.com/content/article/handheld-mapping-of-one-of-europe-s-largest-caves). Range spec 100 m, 30 mm @100 m range accuracy per the ISPRS table below.
- GeoSLAM manual (v1.2 era): start and end in the same position for at least one loop closure, close loops as often as practical, loops better than there-and-back routes (ZEB-REVO manual); too fast forward motion gives too few repeat observations. — [ZEB Horizon manual](https://manuals.online/geoslam/scanner/zeb-horizon/user-manual/1cn35); [ZEB-REVO manual](https://manuals.online/geoslam/scanner/zeb-revo/user-manual/1qpnp) (search summaries, not read in full)
- Heritage-documentation guide for underground sites: finish each loop within 20 minutes; closing loops (start = end) is very important; more loops, better accuracy. — [GIM International, Handheld 3D laser scanning in underground heritage documentation](https://www.gim-international.com/content/article/handheld-3d-laser-scanning-in-underground-heritage-documentation)
- Leica BLK2GO: indoor accuracy +-10 mm (1 sigma) per reseller spec sheet; Leica PDF could not be read. — [ATG BLK2GO specs](https://atgusa.com/product_specs/leica-blk2go-product-specifications/)
- FARO Orbis: up to 5 mm mobile, 2 mm stationary Flash (1 sigma), range 120 m; no distance dependence found. — [FARO Orbis](https://faro.com/en/Products/Hardware/FARO-Orbis-Mobile-Laser-Scanner)

### Cited Findings (independent)
- Gontowa adit (WWII sandstone tunnel, Poland), ~120 m loop, reference Riegl VZ-400i TLS; 2023 comparison of GeoSLAM ZEB Horizon, actuated Velodyne VLP-16 and Livox Horizon systems. Std dev of M3C2 distances to ground truth over full datasets: GeoSLAM 364 mm, Velodyne 281 mm, Livox 232 mm; cross-section and ROI ICP fit std dev ranged 24-114 mm (Velodyne 57-86 mm; Livox and GeoSLAM 24-100 mm). All exceeded 200 mm global std dev; authors attribute this to drift (no global positioning underground) and show double wall / double floor errors in Livox and noisier GeoSLAM sections. Outliers concentrate furthest from the loop closure location. Authors developed drift-compensated evaluation of accuracy/completeness. — [Trybala et al., ISPRS Archives XLVIII-1/W1-2023, 517](https://isprs-archives.copernicus.org/articles/XLVIII-1-W1-2023/517/2023/isprs-archives-XLVIII-1-W1-2023-517-2023.pdf) (read from the PDF text; Table 2)
- Josef mine gallery (~750 m straight tunnel, 3.5 x 3 m), Hovermap ST-X single pass and double pass, reference Leica P40 TLS + MS60 total station network with spherical targets: precise over short sections, errors accumulate over full length mainly along the tunnel axis (slight compression needing scale correction), transverse deviations of tens of cm, double pass reduced transverse error but systematic errors remained; authors recommend periodic control points. Exact RMSE values not retrieved. — [Kremen et al., Acta Montanistica Slovaca 2024](https://actamont.fberg.tuke.sk/pdf/2024/n3/10kremen.pdf) (via search summary; host unreachable)
- 120 m rugged mining tunnel, CTU Prague: Hovermap ST-X two-way scan RMSD 7 mm, max error 17 mm; each scanner measured three times; static Trimble X7 and FARO Focus and SLAM NavVis, GeoSLAM, FARO compared. Reference not stated in the article. Source is Emesent's blog about the independent test (vendor-reported). — [Emesent blog](https://www.emesent.com/blog/2025/01/23/hovermap-outperforms-tls-in-independent-testing)
- Older: ZEB1 tunnel study RMSE 0.15-0.43 m (whole measurement chain); ZEB-REVO planes RMS <10 mm in galleries <10 m. — [GIM International heritage article / search summary](https://www.gim-international.com/content/article/handheld-3d-laser-scanning-in-underground-heritage-documentation) (secondary)
- Hovermap vs tape extensometer (Argyle mine convergence): mean variance 97 mm. — [AusIMM bulletin](https://www.ausimm.com/bulletin/bulletin-articles/improving-convergence-monitoring-using-lidar-data/) (summary only)

### Inferences
- Realistic global error for open or weakly closed handheld traverses of 100-750 m is decimetres (Trybala: 23-36 cm std over a 120 m loop with low-cost systems), versus 0.03 % (3 cm/100 m) vendor drift claim for Hovermap; a dMapper acceptance of <=1 % (1 m per 100 m) is loose, while <=0.1-0.3 % (10-30 cm per 100 m) would be competitive with independent results.
- Relative (local) accuracy of 1-3 cm and global drift are different quantities; criteria should be stated separately.

### Gaps
- No independent numbers found for Stonex X120GO/XVS, CHCNAV RS10, FJD Trion; no Livox-based commercial scanner evaluation underground beyond Trybala's research rig; FARO Orbis and BLK2GO have no independent underground test in my results.
- Josef 750 m RMSE values and Emesent's USIBD accuracy report numbers not obtained.
- No cave (as opposed to mine/tunnel) test with a measured percent drift found.

## 3. Accuracy checking in tunnels, mines, caves: metrics and thresholds

### Takeaway
Literature uses control targets measured with total station or TLS, cloud-to-cloud or M3C2 distance to TLS, cross-section fits and RMSE per axis; underground papers report global errors as std/RMSE in mm to dm and stress that errors depend on distance from the loop closure and along the tunnel axis.

### Cited Findings
- Control points from total station network plus TLS spherical targets, RMSE per X/Y/Z, single vs double pass (Josef mine). — [Kremen 2024](https://actamont.fberg.tuke.sk/pdf/2024/n3/10kremen.pdf)
- M3C2 signed distance to TLS ground truth, cross-section ICP fit, ROI ICP fit, accuracy/completeness curves with drift-compensated clouds (voxel 20 cm and 50 cm completeness comparison). — [Trybala 2023](https://isprs-archives.copernicus.org/articles/XLVIII-1-W1-2023/517/2023/isprs-archives-XLVIII-1-W1-2023-517-2023.pdf)
- Accuracy statements by NavVis, Leica, FARO are 1 sigma in controlled settings; Emesent distinguishes "mapping accuracy" from change detection (+-5 mm). — vendor pages in section 2.
- SLAM vs TLS RMSE about 5 cm on objects in non-tunnel test (Trybala/Szrek 2024). — search summary only, no URL verified.

### Inferences
- Metrics to adopt: per-axis RMSE at targets, C2C/M3C2 mean and 95th percentile after only a rigid alignment, plus the same after removing drift (to separate local from global error), reported against distance from loop closure.

### Gaps
- ATE/RPE-percent drift figures from underground handheld studies and explicit height-error-per-distance or heading-error-per-distance thresholds not found.

## 4. Checks without ground truth and practical hand references

### Takeaway
No-reference metrics exist (mean map entropy, mean plane variance/thickness) but they measure only local crispness, not global geometry; loop misclosure and revisit consistency remain the main global checks; hand measurements were not found as formal literature.

### Cited Findings
- Mean Map Entropy: per-point entropy from covariance of neighbours within a local radius (0.5 m in one paper), averaged over the map; lower is better. Mean Plane Variance paired with it; both no-reference. — [A metric for evaluating 3D reconstruction and mapping performance with no ground truthing](https://arxiv.org/pdf/2101.10402) and [Efficient Continuous-Time SLAM](https://arxiv.org/pdf/1810.06802) (search summaries)
- Plane thickness example (no ground truth): 0.08 m vs 0.12 and 0.14 m, planarity 0.86 vs 0.61 and 0.59 on S3E dataset (indoor/outdoor, not cave). — [LEMON-Mapping](https://arxiv.org/pdf/2505.10018)
- MapEval authors: no-reference metrics only assess local shape, ignoring global geometry. — [MapEval](https://arxiv.org/html/2411.17928v2)
- Cave-survey side: loop misclosure percent and "loops closed and adjusted" (UIS suffix B) are the standard no-ground-truth checks. — sections 1 sources.
- Emesent notes drift is hard to remove in featureless tunnels and recommends control points. — section 2.

### Inferences
- Practical no-ground-truth protocol for dMapper: (a) start and end at the same point, report 3D end-point misclosure in % of path length; (b) scan the loop in both directions and compare clouds (double walls/floors, as seen in Trybala's Livox cloud); (c) crispness: plane thickness and MME on wall patches, compared at first vs last visit; (d) hand references: laser-disto distances between fixed points, compass/clino bearings, plumb/water levels as height checks. These hand references are my suggestion; I found no published protocol for them.

### Gaps
- No source specifying hand-measurement references (disto, tape, plumb lines, water level) for SLAM validation in caves; forward/backward traverse comparison thresholds not found.

## 5. Accuracy requirements for 3D cave/mine models by use

### Takeaway
I found no published numeric accuracy requirements per use (navigation, volume, morphology, hydrology, archaeology).

### Cited Findings
- Volume from LiDAR meshes: errors 0.456 % and 0.394 % against measured volumes (non-cave objects); a cavern-capacity study claims its volume table beats the required standard but the standard's tolerance is not stated. — search summaries ([Chinese Journal of Lasers](https://m.researching.cn/articles/OJe2b3f81172c36f4e))
- Moroccan TLS cave study estimated volume with bounding cuboids (coarse). — [SFB 806 blog](https://www.sfb806.uni-koeln.de/index.php/news-overview/blog/115-project-blog/c2/194-06-2011-project-c2.html)
- Kinect-based stalagmite morphometry overestimated radii 22 % and volumes 44 % (low-cost sensor bias). — search summary, source not opened
- Coverage: complex cave geometry needs many stations to avoid occlusion shadows (Croatian mobile-LiDAR study). — search summary, not opened

### Inferences
- Use-specific thresholds would have to be derived by the developer (e.g. a 1 % global scale error gives about 3 % volume error because volume scales with the cube), not cited.

### Gaps
- All use-specific requirements (navigation, hydrology simulation, archaeology) not found.
