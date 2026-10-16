# Material classification from corrected LiDAR intensity + geometric features: methods, workflows, tools (for cave surveys)

Research note scope: from corrected intensity to material classes. Features, classifiers, uncertainty handling, validation, software, complementary data. Written for implementation in dMapper (C++20, Eigen) or Python post-processing.

Method caveat: in this session several publisher domains (mdpi.com, arxiv.org, copernicus meetingorganizer, HAL, readthedocs) failed DNS from the sandbox, so most findings below come from search-result abstracts/snippets rather than full-text reads. Numbers are quoted as reported in those abstracts; anything not confirmed is flagged.

## Feature sets: corrected-intensity neighbourhood statistics + multi-scale geometric features

### Takeaway
The established recipe is per-point, multi-scale neighbourhood features: eigenvalue features of the local covariance (linearity, planarity, scattering/sphericity, omnivariance, anisotropy, eigenentropy, surface variation/curvature, verticality), computed at several radii (Brodu & Lague CANUPO; Weinmann et al.), combined with statistics of an attribute such as corrected intensity over the same neighbourhoods (3DMASC). Combining spectral (intensity) and geometric features clearly outperforms either alone in the published rock/outcrop studies.

### Cited Findings
- Brodu & Lague (2012), "3D terrestrial lidar data classification of complex natural scenes using a multi-scale dimensionality criterion: applications in geomorphology", ISPRS J. Photogramm. Remote Sens. 68:121–134, DOI 10.1016/j.isprsjprs.2012.01.006 (preprint arXiv:1107.0550). It defines a multi-scale measure of local dimensionality (1D/2D/3D-ness) around each point within spheres of several diameters; vegetation vs. ground separation reached >98% accuracy in their two test cases (riparian vegetation/ground; mountain stream with vegetation, rock, gravel, water). This is the basis of the CloudCompare CANUPO plugin. — [HAL record](https://hal-insu.archives-ouvertes.fr/insu-00700970); [arXiv](https://arxiv.org/abs/1107.0550)
- Weinmann, Jutzi, Hinz & Mallet (2015), "Semantic point cloud interpretation based on optimal neighborhoods, relevant features and efficient classifiers", ISPRS J. Photogramm. Remote Sens. 105:286–304 (DOI 10.1016/j.isprsjprs.2015.01.016 — DOI from the known journal record, not verified on the page in this session). Four-stage pipeline: neighbourhood selection → feature extraction → feature selection → classification; evaluated 7 neighbourhood definitions, 21 geometric features, 7 feature-selection methods, 10 classifiers on 2 benchmarks. Finding: a per-point optimal neighbourhood (eigenentropy-based scale selection) significantly improves results, and a selected feature subset improves quality while cutting time and memory. — [TUM portal](https://portal.fis.tum.de/en/publications/semantic-point-cloud-interpretation-based-on-optimal-neighborhood/); precursor [ISPRS Annals II-3 (2014) 181–188](https://isprs-annals.copernicus.org/articles/II-3/181/2014/); follow-up with 18 low-level 2D/3D features [DGPF 2017](https://dgpf.de/src/tagung/jt2017/proceedings/proceedings/papers/18_DGPF2017_Weinmann_et_al.pdf)
- 3DMASC (multi-attribute, multi-scale, multi-cloud classification): multi-scale features over spherical neighbourhoods including statistics of attributes (intensity etc.) and "dual-cloud" features (ratios/differences between two clouds, e.g. two wavelengths). Most important feature groups: dual-cloud features, lidar return-based attributes averaged over specific scales, and statistics of dimensionality-based and spectral features. Accuracy ~91% (complex multi-class) to ~98% (simpler tasks) with < 2,000 training samples per class. Builds on Brodu & Lague descriptors. Authors include the Rennes Geosciences / LETG group (Letard, Lague et al. — author list not verified in this session). — [arXiv 2401.09481](https://arxiv.org/pdf/2401.09481); [HAL hal-04353070](https://hal-upec-upem.archives-ouvertes.fr/LETG/hal-04353070v3)
- TLS outcrop lithology (Remote Sensing 2025, 17(14):2434): a double-layer random-forest pipeline fusing spectral (reflectance) statistics with geometric descriptors; integrating spectral + geometric features raised macro-F1 from 0.65 (single feature type) to 0.82; adding stratigraphic constraints as post-processing gave 93% overall accuracy, reportedly ahead of SVM, XGBoost and PointNet baselines. (Authors/DOI not retrieved; likely DOI 10.3390/rs17142434 by MDPI pattern — unverified.) — [MDPI PDF](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-17-02434/article_deploy/remotesensing-17-02434.pdf)
- Rockfall-slope materials (EGU 2019 abstract): random forest on multi-scale neighbourhood geometry + slope angle + return-intensity statistics; F-scores ~95% bedrock, 92% vegetation, 85% talus, up to 90% snow; authors report a significant improvement over CANUPO. — [EGU2019-3424](https://meetingorganizer.copernicus.org/EGU2019/EGU2019-3424.pdf)
- Eigenvalue feature catalogue as implemented in common libraries: jakteristics computes eigenvalue sum, omnivariance, eigenentropy, anisotropy, planarity, linearity, PCA1/PCA2, surface variation, sphericity, verticality, normal components (from λ1≥λ2≥λ3 and eigenvectors). pgeof (C++ with Python wrapper; Eigen, nanoflann, Taskflow) computes linearity, planarity, scattering, two verticality variants, normals, length, surface, volume, curvature, optimal neighbourhood size; supports on-the-fly, array and multiscale computation, plus kNN/radius search. — [jakteristics docs](https://jakteristics.readthedocs.io/); [pgeof on PyPI](https://pypi.org/project/pgeof/)
- PDAL `filters.covariancefeatures` computes covariance-based features (default "Dimensionality" set = Linearity, Planarity, Scattering; also Anisotropy, Eigenentropy, Omnivariance, Verticality etc.), `knn` default 10, `feature_set` selectable; density-based options use `filters.optimalneighborhood` output. — [PDAL docs](https://PDAL.io/stages/filters.covariancefeatures.html)
- Cave case: Nováková et al. (2022) classified marble vs. lherzolite in the Gouffre Georges (French Pyrenees) from corrected intensity alone, >84% overall accuracy. A 2026 follow-up (Nováková, Šupinský, Širotník, EGU26-807) uses deep-learning semantic segmentation of cave-floor surfaces (clastic sediments, flowstone, bedrock) from geometric features only. — [ESSD 17:4671 (2025) citing it](https://essd.copernicus.org/articles/17/4671/2025/essd-17-4671-2025.html); [EGU26-807](https://meetingorganizer.copernicus.org/EGU26/EGU26-807.html)

### Inferences
- Concrete feature vector for dMapper (per point or per map voxel), at 3–5 radii (e.g. 0.05, 0.1, 0.2, 0.5, 1.0 m — cave-appropriate, to be tuned): intensity mean/median/std/P10/P90 of corrected intensity; eigenvalue features (λ normalised by sum): linearity (λ1−λ2)/λ1, planarity (λ2−λ3)/λ1, scattering λ3/λ1, omnivariance, eigenentropy, surface variation λ3/Σλ, verticality; normal-angle dispersion; point count/density. These are standard definitions (Weinmann/Demantké lineage); formulas not re-verified from the paper text in this session.
- Roughness at small scale (λ3 or point-to-plane residual std at 1–5 cm) is the geometric feature most likely to separate flowstone (smooth) from clastic sediment (granular) and from fractured bedrock; it is also needed as a covariate for the intensity because roughness changes the angular response (see uncertainty section).
- dMapper already builds NDT cells with covariance; the same Eigen 3×3 eigendecomposition can produce the eigen features at map-cell level cheaply, but per-point multi-radius features need a kd-tree (nanoflann is header-only and compatible with the vendored-dependency policy).

### Gaps
- Exact feature list and scale set of the Remote Sensing 2025 outcrop paper not retrieved (full text unavailable from sandbox).
- No source found quantifying which intensity percentiles (vs mean) matter most; 3DMASC only reports group-level importance.

## Classifiers: thresholds/GMM, random forest, SVM, deep learning; unsupervised; labelled data needs

### Takeaway
Random forest on handcrafted multi-scale features is the dominant, best-documented choice for geoscience point clouds (CANUPO uses a linear/SVM-style projection; 3DMASC and the outcrop/rockfall studies use RF), works with ~1,000–2,000 labelled points per class, and is explainable (feature importance). Simple thresholds on corrected intensity work when only 2–3 contrasting lithologies exist (Nováková 2022, >84%). Deep learning (KPConv, RandLA-Net, PointNet++) needs much more labelled data; no published intensity-input ablation was found.

### Cited Findings
- 3DMASC: random forest, < 2,000 samples per class, 91–98% accuracy; iterative pruning of features and scales using embedded RF metrics. — [arXiv 2401.09481](https://arxiv.org/pdf/2401.09481)
- A 3DMASC developer reports ~1,000 samples per class were enough for very good results (forum comment, not peer-reviewed). The plugin trains with OpenCV's RF (not parallelised, slow); suggested alternative is exporting features via command line and training with scikit-learn. — [CloudCompare forum](https://cloudcompare.org/forum/viewtopic.php?p=24779); [forum](https://mail.cloudcompare.org/forum/viewtopic.php?p=28615)
- Outcrop lithology: double-layer RF beat SVM, XGBoost and PointNet baselines (93% OA with stratigraphic post-processing). — [Remote Sensing 17:2434](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-17-02434/article_deploy/remotesensing-17-02434.pdf)
- Weinmann et al. 2015 compared 10 classifiers; RF is among the efficient, well-performing ones (details not retrieved). — [TUM portal](https://portal.fis.tum.de/en/publications/semantic-point-cloud-interpretation-based-on-optimal-neighborhood/)
- Multispectral ALS (Optech Titan, Finland): RF with multi-channel features separated asphalt/gravel/rocky areas/low vegetation with ~3% OOB error (<1% buildings/trees); multi-channel features more informative than single-channel. Another study: decision tree 93.82% OA multispectral vs 64.66% single-channel. — [ISPRS Archives XLI-B3 (2016) 323](https://isprs-archives.copernicus.org/articles/XLI-B3/323/2016/isprs-archives-XLI-B3-323-2016.pdf); [Remote Sensing 14:238](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-14-00238/article_deploy/remotesensing-14-00238.pdf)
- RandLA-Net (Hu et al., CVPR 2020, arXiv:1911.11236) — efficient large-scale semantic segmentation; its ablations concern architecture (local spatial encoding, attentive pooling), not input channels. KPConv (Thomas et al., ICCV 2019, arXiv:1904.08889) ablations concern kernel design. No intensity-input ablation found for either. — [RandLA-Net CVF](https://openaccess.thecvf.com/content_CVPR_2020/html/Hu_RandLA-Net_Efficient_Semantic_Segmentation_of_Large-Scale_Point_Clouds_CVPR_2020_paper.html); [KPConv overview](https://www.alphaxiv.org/overview/1904.08889)
- Off-road LiDAR intensity-based semantic segmentation (arXiv 2401.01439) uses corrected intensity (I·R²/cos α) as input to segmentation. — [arXiv 2401.01439](https://arxiv.org/html/2401.01439v1)
- Cave deep learning: Nováková et al. EGU26-807 report DL segmentation of cave floor surfaces (sediment, flowstone, bedrock) from geometry; it separates subtly different surfaces and reduces subjectivity of manual digitising (labelled-data amount not given in snippet). — [EGU26-807](https://meetingorganizer.copernicus.org/EGU26/EGU26-807.html)
- Shrubland ALS: K-means on geometric features + intensity to separate rock from shrubs (unsupervised). — [sciencepod summary](https://magazine.sciencepod.net/?p=441166)

### Inferences
- Recommended progression for a cave pipeline: (1) per-survey 1D histogram / GMM on map-aggregated corrected intensity to see whether modes exist (cheap, unsupervised, gives class candidates); (2) RF on intensity statistics + multi-scale geometry with ~1,000–2,000 labelled points per class drawn from several passages; (3) DL only if a large labelled corpus accumulates across surveys.
- RF inference is trivial to embed in C++ (export trees from scikit-learn, or use OpenCV ml as 3DMASC does); training stays in Python. This matches the "Python post-processing" option and keeps dMapper's vendored-only policy (an RF evaluator is a few hundred lines).
- Spatial regularisation after per-point classification (majority filter over neighbours, or a graph-cut/CRF) is what gave the outcrop study its jump from 0.82 F1 to 93% OA (stratigraphic constraints) — an analogous step in caves would be connected-component/region smoothing on the wall surface.

### Gaps
- No published GMM/unsupervised clustering of corrected TLS intensity for rock types was found (search returned only well-log/hyperspectral GMM uses).
- No study found quantifying how much labelled data KPConv/RandLA-Net need for geological classes, nor an ablation of intensity as an input channel.
- PointNet++ not separately researched here.

## Handling intensity uncertainty: weighting, multi-observation aggregation, edges/mixed pixels, incidence-angle and normal errors

### Takeaway
The baseline correction is I_corr = I · R² / cos α (range and Lambertian incidence), with α from PCA normals; but cos α overcorrects at high incidence and rough surfaces deviate from Lambert (Oren–Nayar), normals estimated from noisy points can be badly wrong, and edges produce mixed-pixel range and intensity errors. A mapping pipeline that sees each surface many times can mitigate most of this by aggregating intensity per map cell/surface patch with weights that down-weight large incidence angles, long ranges, edges and uncertain normals, and by rejecting outliers robustly.

### Cited Findings
- Kashani, Olsen, Parrish & Wilson (2015), "A Review of LIDAR Radiometric Processing: From Ad Hoc Intensity Correction to Rigorous Radiometric Calibration", Sensors 15(11):28099–28128, DOI 10.3390/s151128099. Levels: Level 0 raw; Level 1 correction (removes range/incidence effects → pseudo-reflectance); Level 2 normalisation (aligns overlapping strips); higher level calibration to true reflectance. Field lacks standard terminology. — [PMC4701271](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4701271/); level summary via [arXiv 2603.11252](https://arxiv.org/pdf/2603.11252)
- Correction model I(ρ) = I(R,α,ρ)·R²/cos α; cos α = |d·n| with n from PCA normals. One paper found the angle of incidence from estimated normals had an MSE of 0.44 rad (~38°) and trained a network to refine it. — [arXiv 2401.01439](https://arxiv.org/html/2401.01439v1); [arXiv 2508.10398 (Super LiDAR Intensity)](https://arxiv.org/pdf/2508.10398)
- The incidence-angle effect is more complex than the cosine law due to surface roughness; an Oren–Nayar model was proposed (airborne, 2021). A snow study found overcorrection at high incidence angles and suggested a 40° incidence threshold. — [UNT digital library, airborne correction](https://digital.library.unt.edu/ark:/67531/metadc1852165/m2/1/high_res_d/Airborne-lidar-intensity-correction-based-on-a-new-method.pdf); [Remote Sensing 2020 (DOAJ)](https://doaj.org/article/f0b92d0514a6432eaad3db7fd9351417)
- Krooks et al. (2013, ISPRS Annals II-5/W2:145): distance and incidence-angle effects are separable and lab-measured correction values can correct field TLS data. Tan & Cheng (2016, Remote Sensing): incidence angle and distance are the dominant effects on FARO Focus3D intensity; reference-target interpolation method; raw intensity should not be used without correction. — [ISPRS Annals](https://isprs-annals.copernicus.org/articles/II-5-W2/145/2013/); [ECNU record](https://pure.ecnu.edu.cn/en/publications/correction-of-incidence-angle-and-distance-effects-on-tls-intensi/)
- Nováková et al. (2022), "Correcting laser scanning intensity recorded in a cave environment for high-resolution lithological mapping: A case study of the Gouffre Georges, France", Remote Sens. Environ. 280:113210 (DOI likely 10.1016/j.rse.2022.113210 — not verified). Correction covers acquisition geometry, surface micromorphology and atmospheric effects; wet surfaces bias intensity low and complicate discrimination. An EGU2020 presentation of this work stresses filtering/denoising before per-rock thresholds because surface condition, water content and active karst processes affect the signal. — [Nováková 2022 via search/HAL](https://hal-bioemco.ccsd.cnrs.fr/GR-3T/insu-03752599v1); [EGU2020-18742](https://presentations.copernicus.org/EGU2020/EGU2020-18742_presentation.pdf)
- Mixed pixels: at edges the beam footprint straddles two surfaces; distances near edges are systematically short or long depending on edge type; residuals correlate with intensity, incidence angle and spot size (Hartmann & Neumann, FIG 2022). Footprint elongates with incidence angle. — [FIG 2022 slides](https://www.fig.net:443/resources/proceedings/fig_proceedings/fig2022/ppt/pe1.02/PE1.02_hartmann_neumann_11443_ppt.pdf); [3DCP review arXiv 2503.07472](https://arxiv.org/pdf/2503.07472); [ICCCBE 2018 abstract](https://programme.exordo.com/icccbe2018/delegates/presentation/338/)
- MLS radiometric calibration with little overlap: Laasch, Medic & Wieser (ETH, ISPRS Archives XLVIII-G-2025:845) — automatic in-situ calibration compensating distance and incidence angle, global and per-segment angle strategies, since TLS methods rely on overlapping discrete scans. — [ISPRS Archives 2025](https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/845/2025/isprs-archives-XLVIII-G-2025-845-2025.pdf)
- Multi-sensor reflectivity map fusion: Castorena (arXiv 1611.09203) fuses reflectivity gradients from several scanners via sparse regularised least squares and Poisson reconstruction, removing artefacts and stitching without per-sensor adjustment. Learning calibration parameters from registered clouds because multiple scanners disagree on remission of the same surface. — [arXiv 1611.09203](https://arxiv.org/pdf/1611.09203); [IEEE record](https://www.infona.pl/resource/bwmeta1.element.ieee-art-000007139472)

### Inferences
- dMapper-specific design (map-level aggregation): after the final trajectory, for each surface voxel/NDT cell collect all observations (I, R, α, sensor time, beam direction). Use a normal from the aggregated map (many scans, much better than single-scan PCA normal), compute I·R²/f(α) with f either cosine with α ≤ ~40–60° cutoff or an empirically fitted f(α) per sensor (fit from the data itself: for cells seen at many angles, regress log I vs α — a self-calibration like Laasch et al.). Aggregate with a robust estimator (median or Huber-weighted mean) and weights w ∝ cos α / σ_I²(R), reject observations whose point lies near a depth discontinuity (mixed pixels; detect via range jumps between neighbouring beams or cell planarity low/scattering high), and store mean, std, n_obs, and angular coverage. Std across observations is itself a feature (and a QC flag).
- Normal-error propagation: d(cos α)/dα = −sin α, so relative intensity error from a normal error δ is ≈ tan α · δ; at α = 60° a 5° normal error gives ~15% error, at 20° only ~3%. This is a strong argument to weight by cos α and cap α. (Derivation, not from a source.)
- Range-dependent effects of near-range receivers (many sensors deviate from R² at short range, which matters in narrow cave passages) should be calibrated empirically per sensor model; Kashani et al. and Krooks et al. both support lab/target-based range curves.
- Wetness is the dominant confounder in caves; record it per survey and avoid cross-survey thresholds unless conditions match.

### Gaps
- No source quantifying normal-estimation error propagation into corrected intensity for TLS/MLS was found.
- No open-source GitHub repository for TLS/MLS intensity correction was found in searches (only papers); the arXiv off-road paper may have code — not verified.
- Sensor-specific near-range behaviour for Livox/Ouster/Hesai-class sensors (the likely dMapper sensor) not researched here.

## Validation strategies and reported accuracies

### Takeaway
Validation in this literature uses manually labelled points/regions (interpreted from photos, field mapping, or geological maps), reported as overall accuracy, per-class F-score/macro-F1, kappa and confusion matrices; reported accuracies are ~84% (cave lithology from intensity only), 85–95% (slope materials, RF), 82%→93% (outcrop lithology, RF + constraints), 91–98% (3DMASC).

### Cited Findings
- Nováková 2022 cave lithology: >84% overall accuracy for marble vs lherzolite from corrected intensity. — [HAL](https://hal-bioemco.ccsd.cnrs.fr/GR-3T/insu-03752599v1)
- Rockfall slope: per-class F-scores 95/92/85/90% (bedrock/vegetation/talus/snow). — [EGU2019-3424](https://meetingorganizer.copernicus.org/EGU2019/EGU2019-3424.pdf)
- Outcrop lithology: macro-F1 0.65 → 0.82 (feature fusion); 93% OA with stratigraphic constraints. — [Remote Sensing 17:2434](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-17-02434/article_deploy/remotesensing-17-02434.pdf)
- 3DMASC: 91–98%, <2,000 samples/class; lidar-platform Python module (`classification.cc_3dmasc`) provides train, multi-metric evaluation, test on held-out data, feature-importance and SHAP plots. — [arXiv 2401.09481](https://arxiv.org/pdf/2401.09481); [lidar-platform docs](https://lidar-platform.readthedocs.io/en/latest/3dmasc.html)
- Hyperspectral GMM mineral mapping validated against expert-labelled drill core (analogue for unsupervised validation). — [CVPRW 2021 Gao et al.](https://openaccess.thecvf.com/content/CVPR2021W/PBVS/papers/Gao_Generalized_Unsupervised_Clustering_of_Hyperspectral_Images_of_Geological_Targets_in_CVPRW_2021_paper.pdf)

### Inferences
- For caves: label from co-registered photos plus field hand samples at marked spots; validate with spatially blocked cross-validation (hold out whole passages), because random point splits inflate accuracy through spatial autocorrelation. Report confusion matrix, per-class F1, and accuracy as function of incidence angle and range bins (diagnoses residual correction errors).
- Controlled reference: scan Spectralon-type or known-reflectance targets in the cave at several ranges/angles to fix the range curve and check repeatability between surveys.

### Gaps
- No study found using handheld spectrometer readings at the laser wavelength (905/1550 nm) as ground truth for cave intensity; not confirmed whether Nováková 2022 did.
- Spatial-block cross-validation use in these specific papers not verified.

## Tools and software

### Takeaway
CloudCompare (scalar fields, CANUPO, q3DMASC with command line), PDAL (`filters.covariancefeatures`, `filters.optimalneighborhood`), Python libraries jakteristics and pgeof for features, scikit-learn for RF; Open3D/PCL provide normals and kd-trees. Nothing ready-made was found for cave-specific intensity correction.

### Cited Findings
- q3DMASC (CloudCompare plugin): needs a labelled core cloud + up to 3 accessory clouds; parameter text file lists clouds, scales and features; saves a reusable classifier; RF via OpenCV; CLI options `3DMASC_CLASSIFY`, `ONLY_FEATURES`, `KEEP_ATTRIBUTES`, `SKIP_FEATURES`; GUI harder than CANUPO. — [lidar-platform docs](https://lidar-platform.readthedocs.io/en/latest/3dmasc.html); [CC forum](https://cloudcompare.org/forum/viewtopic.php?p=24779); [CC forum](https://mail.cloudcompare.org/forum/viewtopic.php?p=28820)
- CANUPO: multi-scale dimensionality classifier suite (original authors Brodu & Lague); a GitHub mirror exists (third-party). — [arXiv 1107.0550](https://arxiv.org/abs/1107.0550); [GitHub mirror](https://github.com/OrangeMo0n/canupo)
- PDAL `filters.covariancefeatures` (features listed above). — [PDAL](https://PDAL.io/stages/filters.covariancefeatures.html); [PDAL filters index](https://pdal.io/stages/filters.html)
- jakteristics (Cython/LAPACK, inspired by CloudCompare's features), pgeof (C++ core: Eigen, nanoflann, Taskflow; v0.3.4, 2025-10). — [jakteristics](https://jakteristics.readthedocs.io/); [pgeof](https://pypi.org/project/pgeof/); [piwheels](https://www.piwheels.org/project/pgeof)
- MATLAB `extractEigenFeatures` (Lidar Toolbox) also exists. — [MathWorks](https://www.mathworks.com/help/lidar/ref/extracteigenfeatures.html)

### Inferences
- For dMapper: pgeof's C++ core (Eigen + nanoflann) is the closest model for an in-pipeline feature extractor; its dependencies match dMapper's vendored-Eigen setup (licence check needed before vendoring).
- Practical workflow: dMapper exports merged LAZ with extra dims (corrected intensity mean/std/n_obs, incidence angle, range) → CloudCompare/q3DMASC or Python (pgeof + scikit-learn) for feature computation and RF training → classifier applied in batch.

### Gaps
- Open3D, PCL, lidR and LAStools intensity-related features not checked in this session (known capabilities: normals/kd-tree in Open3D/PCL; lidR/LAStools airborne oriented) — unverified here.

## Complementary data: RGB/thermal, multispectral LiDAR, spectrometers

### Takeaway
Multi-wavelength intensity is the strongest single improvement in the literature (dual-cloud ratio features top 3DMASC's importance; multispectral raises accuracy from ~65% to ~94% in one urban study). In caves, co-registered RGB (with artificial light) is the practical complement; Nováková's workflow reportedly uses intensity and colour.

### Cited Findings
- 3DMASC dual-cloud (green + NIR) features are among the most important. — [arXiv 2401.09481](https://arxiv.org/pdf/2401.09481)
- Optech Titan multispectral: multi-channel features more informative than single-channel; decision-tree OA 93.82% vs 64.66% single-channel; object-based multispectral classification >90% OA. — [ISPRS 2016](https://isprs-archives.copernicus.org/articles/XLI-B3/323/2016/isprs-archives-XLI-B3-323-2016.pdf); [Remote Sensing 14:238](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-14-00238/article_deploy/remotesensing-14-00238.pdf); [Remote Sensing 13:4118 (crossref)](https://api.crossref.org/works/10.3390%2FRS13204118)
- Cave bedrock classification combining intensity and colour attributes (secondary summary of Nováková et al. 2022, not confirmed from paper). — [ESSD 17:4671](https://essd.copernicus.org/articles/17/4671/2025/essd-17-4671-2025.html)
- Speleothem (ceiling ornament) classification from LiDAR or photogrammetry point clouds by morphology, location and development process (thesis, UFC Brazil 2022). — [UFC repository](https://repositorio.ufc.br/handle/riufc/69727?locale=en)

### Inferences
- With a single-wavelength sensor, the best extra "channel" is a second observation condition (e.g. the per-cell angular response slope, a proxy for roughness/gloss — wet flowstone is specular) plus RGB from a synchronized camera.
- Thermal cameras could flag wet/seepage zones (evaporative cooling) to mask intensity biased by water; not supported by a found source.

### Gaps
- No source found on thermal imagery or handheld (field) spectrometers for cave LiDAR material validation.
- Whether 905 nm vs 1550 nm sensors differ in water sensitivity for cave rocks was not researched (1550 nm is strongly absorbed by water — general physics, unverified here).
