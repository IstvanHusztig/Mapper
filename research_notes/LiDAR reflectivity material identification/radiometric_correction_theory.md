# Radiometric correction of LiDAR intensity (range and incidence-angle normalization), focus on 905 nm ToF scanners

Note on method: web search worked, but every full-text fetch (MDPI, PMC, ISPRS, Copernicus, arXiv, FIG) was refused by this session's egress proxy. Findings marked with a [Source] were confirmed through search-result abstracts and bibliographic records. Formulas that are standard textbook material but could not be checked against the full text in this session are labelled "(standard form, not verified against full text here)". Treat those as reliable in structure, and check symbols and constants against the cited paper before implementing.

## 1. The LiDAR range equation for intensity, and short-range deviations

### Takeaway
For an extended diffuse (Lambertian) target that fills the beam, the received power is P_r ∝ ρ·cos(α)/R² (the beam-area R² cancels one R² of the point-target R⁴ law). Real TLS/MLS instruments deviate strongly at short range (below about 1–10 m depending on the instrument) because of near-field optics (overlap of transmitter and receiver fields of view, defocus), brightness reducers, logarithmic amplifiers/AGC and detector saturation. So the range term is in practice estimated empirically (polynomial, piecewise, or as a "laser transmission function"), not taken as 1/R².

### Cited Findings
- Höfle & Pfeifer (2007), "Correction of laser scanning intensity data: data and model-driven approaches", ISPRS J. Photogramm. Remote Sens. 62(6):415–433, doi:10.1016/j.isprsjprs.2007.05.008. They name three main influences on the backscattered power: spherical loss (range), topographic effects (incidence angle) and atmospheric effects. The data-driven method fits a global correction function by least squares over predefined homogeneous areas; the model-driven method derives a correction factor per point from the radar (LiDAR) equation. Both reduced the disturbing effects to below the noise level of about 10% — [Heidelberg catalogue record](https://heibib.ub.uni-heidelberg.de/search/Record/1885638124); [Höfle dissertation 2007](https://archiv.ub.uni-heidelberg.de/volltextserver/36930/1/hoefle_dissertation_2007.pdf). (One dissertation snippet garbles the "10%" figure, so check the full text.)
- Range equation, as used by Höfle & Pfeifer (2007), Jutzi & Gross (2009) and Kashani et al. (2015) (standard form, not verified against full text here):
  - P_r = P_t · D_r² · η_sys · η_atm · σ / (4π · R⁴ · β_t²), where P_t is the transmitted power, D_r the receiver aperture diameter, β_t the beam divergence, η_sys the system transmission, η_atm the atmospheric transmission and σ the backscatter cross-section.
  - For an extended Lambertian target that fills the footprint: σ = π · ρ · R² · β_t² · cos α, so P_r = P_t · D_r² · η_sys · η_atm · ρ · cos α / (4 R²).
  - Correction to a reference range R_ref and normal incidence: I_c = I_raw · (R/R_ref)² · 1/cos α (· 1/η_atm if not negligible).
  - For a non-extended target smaller than the footprint (a wire, a leaf, an edge) σ does not grow with R², and P_r ∝ 1/R⁴. For a linear target (a wire), P_r ∝ 1/R³.
- Kashani, Olsen, Parrish & Wilson (2015), "A Review of LIDAR Radiometric Processing: From Ad Hoc Intensity Correction to Rigorous Radiometric Calibration", Sensors 15(11):28099–28128, doi:10.3390/s151128099. The review notes that terminology ("normalization", "correction", "calibration") is used inconsistently. Its processing levels: Level 0 raw intensity; Level 1 intensity correction (range, incidence angle → pseudo-reflectance); Level 2 intensity normalization (scale/shift so that overlapping strips/scans agree); Level 3 rigorous radiometric calibration to true reflectance — [Sensors/PMC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4701271/); level definitions as summarized by [arXiv 2603.11252](https://arxiv.org/pdf/2603.11252). The Level 3 definition was not confirmed from the text.
- Kaasalainen, Jaakkola, Kaasalainen, Krooks & Kukko (2011), "Analysis of Incidence Angle and Distance Effects on Terrestrial Laser Scanner Intensity: Search for Correction Methods", Remote Sens. 3(10):2207–2221, doi:10.3390/rs3102207. It studied range and incidence-angle effects for several TLS instruments and targets (Spectralon 99%, tarps, sand, gabbro). It stresses that knowing whether an instrument has near-distance reducers or logarithmic amplifiers (for small reflectance) is "of cardinal importance". An earlier Kaasalainen et al. (2009) paper is credited with finding a brightness reducer acting below about 1 m. The Faro LS HE80's logarithmic amplifier for low reflectance was examined in a 2008 study — [TUNI research portal](https://researchportal.tuni.fi/en/publications/analysis-of-incidence-angle-and-distance-effects-on-terrestrial-l/); secondary summary in [Mukupa et al., FIG 2017](https://fig.net:443/resources/monthly_articles/2017/mukupa_etal_september_2017.asp).
- Krooks et al. (2013, ISPRS Annals II-5/W2): distance and incidence-angle effects do not interact, so they can be corrected as separable factors f(R)·g(α), and laboratory-derived corrections can be applied to field data. A 4-step Spectralon plate (99/50/25/12%) was used as reference — [ISPRS Annals II-5-W2-145-2013](https://isprs-annals.copernicus.org/articles/II-5-W2/145/2013/isprsannals-II-5-W2-145-2013.pdf).
- Fang, Huang, Zhang & Li (2015), "Intensity Correction of Terrestrial Laser Scanning Data by Estimating Laser Transmission Function", IEEE TGRS 53(2):942–951, doi:10.1109/TGRS.2014.2330852 (online 2014). It models the near-range deviation as a range-dependent instrument "laser transmission function" that multiplies the 1/R² law. The details of the function's form were not retrieved — [visionbib listing](https://visionbib.com/bibliography/active672li1.html); [Crossref](https://api.crossref.org/works/10.3390%2FRS9040356).
- Tan & Cheng (2015), "Intensity data correction based on incidence angle and distance for terrestrial laser scanner", J. Appl. Remote Sens. 9(1):094094, doi:10.1117/1.JRS.9.094094. They separate reflectance, incidence angle and distance, and write intensity as a product of polynomials in each, I = f₁(ρ)·f₂(cos α)·f₃(R), justified by the Weierstrass approximation theorem. Polynomial degrees and coefficients are fitted by least squares on diffuse targets of known reflectance. The authors claim it suits all TLS instruments — [SPIE](https://biomedicaloptics.spiedigitallibrary.org/journals/journal-of-applied-remote-sensing/volume-9/issue-01/094094/Intensity-data-correction-based-on-incidence-angle-and-distance-for/10.1117/1.JRS.9.094094.full); [WHU record](https://ch.whu.edu.cn/article/doi/10.13203/j.whugis20150502).
- Tan, Cheng, Ding & Zhang (2016), IEEE JSTARS, "Intensity data correction for the distance effect in terrestrial laser scanners". The Lambertian cosine law works well for incidence angle, but the distance effect is more complex and system-specific. After correction, the coefficient of variation (CV) on homogeneous targets fell to about 0.19× (Faro Focus3D 120) and about 0.07× (Focus3D X330) of the raw CV — [PolyU](https://research.polyu.edu.hk/en/publications/intensity-data-correction-for-the-distance-effect-in-terrestrial-/); [ECNU](https://pure.ecnu.edu.cn/en/publications/intensity-data-correction-for-the-distance-effect-in-terrestrial-/).
- Tan & Cheng's Faro work also used linear interpolation of the intensities of reference targets scanned at a grid of incidence angles and distances, a look-up-table approach — [ECNU](https://pure.ecnu.edu.cn/en/publications/correction-of-incidence-angle-and-distance-effects-on-tls-intensi/).
- Tan & Cheng (2020), IEEE GRSL: the distance-intensity function is estimated from naturally homogeneous targets after first removing the incidence-angle effect, with no artificial targets. CV reduced by 52%. A Riegl VZ-4000 intertidal study reports about 54% CV reduction — [ECNU](https://pure.ecnu.edu.cn/en/publications/distance-effect-correction-on-tls-intensity-data-using-naturally-/).
- Xu, Xu et al. (2017), "Terrestrial Laser Scanning Intensity Correction by Piecewise Fitting and Overlap-Driven Adjustment", Remote Sens. 9(11):1090, doi:10.3390/rs9111090. Distance effect by piecewise fitting (separate functions for the near and far range); incidence angle by overlap-driven adjustment with the Oren–Nayar model and a surface-roughness parameter. Riegl VZ-400i: retrieved reflectance within 3% of spectrometer measurements — [MDPI](https://www.mdpi.com/2072-4292/9/11/1090); [doi](https://0-doi-org.brum.beds.ac.uk/10.3390/rs9111090).
- Mukupa et al. (2017, FIG): a hybrid correction for near-distance effects (scans at 1–6 m) and surface roughness (Oren–Nayar), on concrete with a Leica HDS7000. No single TLS intensity correction is accepted as standard — [FIG article](https://fig.net:443/resources/monthly_articles/2017/mukupa_etal_september_2017.asp); [FIG slides](https://fig.net/resources/proceedings/fig_proceedings/fig2017/ppt/ts03f/TS03F_mukupa_roberts_et_al_8547_ppt.pdf).
- Off-road/automotive LiDAR intensity literature also notes that the range-intensity relation is not inverse-square at close range — [Moonlight review of "Off-Road LiDAR Intensity Based Semantic Segmentation"](https://www.themoonlight.io/review/off-road-lidar-intensity-based-semantic-segmentation).
- Automotive multi-channel LiDAR patents fit a per-channel, per-power-level linear model (multiplier + bias) from targets of known reflectance at several distances and orientations. This implies that sensors with auto power/gain switching need a separate calibration per power level — [US patent "Lidar intensity calibration" 11067676](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11067676).

### Inferences
- Near-range mechanisms, from the optics (general LiDAR knowledge, consistent with the sources above):
  - **Overlap function O(R):** in a biaxial (side-by-side) transmitter/receiver, the receiver field of view only fully covers the laser spot beyond a crossover range, so O(R) rises from 0 to 1. Even coaxial systems defocus the return at short range. This makes intensity fall, not rise, at very close range.
  - **Brightness reducers / ND filters** (Kaasalainen 2009, below about 1 m) and **AGC or logarithmic amplifiers** compress strong returns.
  - **APD saturation and pulse pile-up** clip bright, close targets.

  The resulting raw curve I(R) on a constant target usually rises to a peak at a few metres, then decays roughly as R⁻² (sometimes R⁻ᵃ with a ≠ 2 after the internal non-linear mapping).
- **Practical model:** I_raw = g(η(R) · ρ · h(α) / R²), where:
  - η(R) is a near-range factor, fitted as a polynomial or piecewise function, or tabulated;
  - g is the instrument's non-linear intensity mapping (often log or vendor-specific "reflectivity" encoding).

  Fit η(R)/R² jointly as one empirical function f(R) per channel (and per gain/power state), as Tan & Cheng and Xu et al. do.
- **Velodyne/Ouster/Livox-class 905 nm sensors** (per-beam lasers, different channels, often onboard "reflectivity" pre-correction):
  - Each beam needs its own f(R), and per-beam gain differences dominate unless calibrated (see section 4, Levinson & Thrun).
  - In a cave survey (ranges 0.5–30 m), much of the data lies in the non-1/R² near zone. An empirical f(R) per beam is essential there, and 1/R² alone will be badly wrong below a few metres.
- **Mixed models** combine a parametric optical overlap term (e.g. O(R) = 1 − exp(−(R/R₀)^k)) with 1/R². I found none published for 905 nm automotive sensors in this session. Polynomial or piecewise fits are the established practice.

### Gaps
- Could not retrieve the exact functional form of Fang et al.'s (2015) transmission function or the coefficients and polynomial degrees of Tan & Cheng (full texts blocked).
- No published near-range curve for a specific 905 nm automotive/robotic sensor (Velodyne, Ouster, Hesai, Livox) was found in this session. Ouster documentation (docs.ouster.com) was blocked by the proxy.
- Kashani et al.'s exact Level 3 definition, and their full parameter list, were not confirmed from full text.

## 2. Incidence-angle effect: Lambertian vs non-Lambertian models, normal estimation

### Takeaway
The Lambertian cos α law is the default and works reasonably for matte surfaces up to about 60–70°. Rough natural surfaces (rock, concrete) deviate, and the Oren–Nayar model with a roughness parameter σ fits them better; empirical cos^n or polynomial-in-cos α fits are also common. The incidence angle comes from a local normal (PCA of the neighbourhood covariance). Normal errors matter most at high incidence angles, where 1/cos α amplifies them, so points above about 70–80° are usually discarded or down-weighted.

### Cited Findings
- Tan et al. (2016) report that several studies found the Lambertian cosine law adequate for TLS incidence-angle correction — [PolyU](https://research.polyu.edu.hk/en/publications/intensity-data-correction-for-the-distance-effect-in-terrestrial-/).
- The LiDAR range equation gives backscatter ∝ cos α, and Lambert's law is widely used for intensity correction; the same paper treats the hyperspectral case — [Remote Sens. 12:2855 (2020)](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-12-02855/article_deploy/remotesensing-12-02855.pdf).
- Carrea, Abellan, Humair, Matasci, Derron & Jaboyedoff (2016), "Correction of terrestrial LiDAR intensity channel using Oren-Nayar reflectance model: An application to lithological differentiation", ISPRS J. Photogramm. Remote Sens. 113:17–29, doi:10.1016/j.isprsjprs.2015.12.004. The companion EGU 2015 abstract states that the simple cosine law is insufficient for rough rock surfaces and proposes a simplified Oren–Nayar model — [EGU2015-10643](https://meetingorganizer.copernicus.org/EGU2015/EGU2015-10643.pdf?pdf); [UA record](https://observatorio-cientifico.ua.es/documentos/652b2ab2ebee473633a3aa5a).
- Oren & Nayar (1995), "Generalization of the Lambertian model and implications for machine vision", IJCV 14:227–251, doi:10.1007/BF01539553, cited as the roughness model in the TLS/hyperspectral literature — [Sensors 21(9):2960](https://www2.mdpi.com/1424-8220/21/9/2960).
- Mukupa et al. (2017): mm- to cm-scale surface irregularities comparable to the footprint partly cancel the incidence-angle effect, because some facets always face the beam. They report a claim that scattering-angle effects can be neglected when roughness is comparable to the spot size — [FIG 2017](https://fig.net:443/resources/monthly_articles/2017/mukupa_etal_september_2017.asp).
- Xu et al. (2017) use Oren–Nayar with a fitted roughness parameter in an overlap-driven adjustment, reaching <3% reflectance error (Riegl VZ-400i) — [MDPI](https://www.mdpi.com/2072-4292/9/11/1090).
- Jutzi & Gross (2009), "Normalization of LiDAR intensity data based on range and surface incidence angle", ISPRS Archives XXXVIII-3/W8: normalization for beam divergence, incidence angle and atmospheric attenuation — [ISPRS PDF](https://www.isprs.org/proceedings/XXXVIII/3-W8/papers/213_laserscanning09.pdf). Jutzi & Gross (2010), "Investigations on surface reflection models for intensity normalization in ALS data", compare reflection models. The surface orientation comes from the eigenvectors of the covariance matrix of nearby points (PCA normal). After normalization the intensity depends on the material only, not the incidence angle — [TUM portal](https://portal.fis.tum.de/en/publications/investigations-on-surface-reflection-models-for-intensity-normali/).
- Laasch, Medic & Wieser (ETH Zürich, 2025, ISPRS Archives XLVIII-G-2025, doi:10.5194/isprs-archives-XLVIII-G-2025-845-2025) adapt an automatic in-situ radiometric calibration from TLS to MLS. They estimate angle-of-incidence (AOI) and distance compensation functions with little or no overlap:
  - "global" AOI compensation: one function for the whole scan, using local reflectance estimation with modified filtering;
  - "local" AOI compensation: one function per segment, ideally a single material.

  The TLS companion paper (ISPRS J. 228:648–665, 2025) outputs a configuration-independent intensity rather than "reflectance", because reflectance itself depends on the beam–surface geometry — [ISPRS Archives](https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/845/2025/); [TU Wien repository](https://repositum.tuwien.at/handle/20.500.12708/218316?mode=full).

### Inferences
Model formulas (standard forms, not verified against full text here):
- **Lambert:** I ∝ ρ cos α.
- **Empirical cos^n:** I ∝ ρ cos^n α. n < 1 means a rough or retro-reflective-leaning surface; n > 1 a specular-leaning one.
- **Oren–Nayar (monostatic case):**
  - For LiDAR the source and receiver coincide (θ_i = θ_r = α, azimuth difference 0), and the model reduces to I ∝ (ρ/π) cos α · [A + B · sin α · tan α].
  - A = 1 − 0.5 σ²/(σ² + 0.33) and B = 0.45 σ²/(σ² + 0.09), with σ the facet slope standard deviation (rad).
  - Carrea et al. use a "simplified" variant.
  - The model gives a weaker fall-off than cos α at large α, which matches the observed behaviour of rock.
- **Phong / specular lobe:** I ∝ k_d cos α + k_s cos^m(2α) for the monostatic geometry, where the mirror direction is 2α away from the beam. This matters for wet or polished surfaces: wet cave walls, flowstone, calcite.
- **Beckmann (microfacet):** a Gaussian slope-distribution specular term. Rarely used for TLS.
- **Lommel–Seeliger:** I ∝ ρ cos α_i / (cos α_i + cos α_r), which equals ρ/2 (constant) for the monostatic geometry. It describes low-albedo, volume-scattering media (regolith, dark porous rock) that show almost no incidence-angle dependence. I found no TLS paper applying it in this session.

Normal estimation:
- PCA on k nearest neighbours or a radius; the normal is the eigenvector of the smallest eigenvalue.
- Too small a neighbourhood: range noise (mm–cm for 905 nm ToF) gives noisy normals.
- Too large a neighbourhood: corners and curvature are smoothed away.
- Rule of thumb: a radius a few times the point spacing and above about 10× the range noise, e.g. 5–15 cm for a mobile 905 nm scanner with ±2–3 cm noise.
- Error propagation: δ(1/cos α)/(1/cos α) = tan α · δα. A 5° normal error gives about 0.4% intensity error at 10°, 4.4% at 45°, 15% at 60°, 50% at 80°.
- Common practice: drop points with α > 70–75°, or weight them by cos α.

On rough surfaces the "geometric" normal at the neighbourhood scale is not the normal of the facets that actually reflect within the footprint. That is why a fitted roughness parameter (Oren–Nayar σ, or a per-material cos^n exponent) replaces pure geometry. The beam footprint (divergence about 1–3 mrad for 905 nm spinning sensors, i.e. 1–3 cm at 10 m) sets the facet scale.

For mobile mapping in a cave, the per-segment (local) AOI compensation of Laasch et al. 2025 is the closest published analogue: an in-situ, target-free, low-overlap method.

### Gaps
- Could not retrieve Carrea et al.'s fitted roughness values or their lithology classification accuracies.
- No direct published comparison of cos^n vs Oren–Nayar vs Lambert for 905 nm automotive sensors was found.
- No paper quantifying neighbourhood size vs normal error for incidence-angle correction was retrieved. The error-propagation figures above are computed, not cited.

## 3. Atmospheric attenuation at 905 nm (indoors, caves, humidity, fog, droplets)

### Takeaway
In clear air, attenuation at 905 nm over tens of metres is negligible: extinction is about 0.1 km⁻¹ or less in clear air, so the two-way loss over 30 m is under about 1%. Fog, mist or spray changes that. Extinction in fog is nearly wavelength-independent below about 0.5 km visibility, and adds false near returns and intensity loss. In a humid cave, mist and droplets on the sensor window are the realistic radiometric risks, not gaseous absorption.

### Cited Findings
- **Kim visibility model:** specific attenuation γ(λ) = (3.91/V) · (λ/550 nm)^(−q), with V the visibility in km (often quoted in dB/km as 17/V · (λ/550)^(−q)).
  - q = 1.6 for V > 50 km;
  - q = 1.3 for 6–50 km;
  - q = 0.16V + 0.34 for 1–6 km;
  - q = V − 0.5 for 0.5–1 km;
  - q = 0 for V < 0.5 km (dense fog: no wavelength dependence).

  The prefactor and unit conventions differ between papers — [Radioengineering 19(2) 2010](https://www.radioeng.cz/fulltexts/2010/10_02_223_227.pdf); [Applied Sciences 2019 (DOAJ)](https://doaj.org/article/e3d51af263e646b4ad24c22cced59866).
- Duthon et al. (2019): less than 10% difference between the 905 nm and 1550 nm extinction coefficients in fog for the same emitted power — summarized via [Applied Sciences 2019 (DOAJ)](https://doaj.org/article/e3d51af263e646b4ad24c22cced59866).
- "What happens to a ToF LiDAR in fog?" (arXiv 2003.06660) studies 905 nm ToF sensors in fog. A Velodyne HDL-64E (905 nm) study modelled fog attenuation of intensity with an empirical visibility-based model — [arXiv 2003.06660](https://arxiv.org/pdf/2003.06660); [VTT, spectral attenuation in artificial fog](https://cris.vtt.fi/en/publications/spectral-attenuation-in-low-visibility-artificial-fog-experimenta/).
- Jutzi & Gross (2009) include atmospheric attenuation in their normalization, and Höfle & Pfeifer (2007) list it as one of the three main influences (relevant mainly for airborne ranges of hundreds of metres to km) — [ISPRS 2009](https://www.isprs.org/proceedings/XXXVIII/3-W8/papers/213_laserscanning09.pdf); [Heidelberg record](https://heibib.ub.uni-heidelberg.de/search/Record/1885638124).

### Inferences
- **Two-way transmission:** η_atm = exp(−2 γ R).
  - Clear cave air (V ≫ 10 km, γ < 0.4 km⁻¹): η_atm > 0.976 at 30 m. Negligible against the other error terms.
  - Light mist (V = 1 km, γ ≈ 3.9 km⁻¹): η_atm ≈ 0.79 at 30 m and 0.92 at 10 m. A range-dependent bias of 8–20%, comparable to material contrasts.
  - Fog (V = 0.1 km): η_atm ≈ 0.10 at 30 m. Intensity unusable.
- Water vapour absorption bands lie near 940 nm and 1130 nm. 905 nm sits on the edge of the 940 nm band, so laser drift with temperature (about 0.25 nm/K for edge-emitting 905 nm diodes) could in principle shift absorption slightly. Over a few tens of metres this remains negligible (general knowledge, not verified here).
- Droplets or condensation on the scanner window attenuate every return by the same factor at all ranges, plus scattering near the window. Breath or exhaust plumes in a cave give localized false returns. These show up as a per-frame gain change, best handled by relative (overlap) normalization rather than by an atmospheric model.

### Gaps
- No cave-specific or underground-mine measurement of 905 nm intensity attenuation was found.
- The original Kim et al. (2001) paper was not retrieved; coefficients come from secondary sources with differing conventions.

## 4. Calibration procedures: reference targets, relative vs absolute, self-calibration

### Takeaway
Absolute (Level 3) calibration scans reference panels of known reflectance (Spectralon 99/50/25/12% or similar) over a grid of ranges and angles, and fits f(R), g(α) (separable, per Krooks 2013) plus the instrument's intensity-to-reflectance mapping. Relative (Level 2) calibration uses overlaps or homogeneous natural surfaces with no targets: overlap-driven adjustment (Xu 2017, Laasch 2025), natural homogeneous targets (Tan & Cheng 2020), or, for multi-beam sensors, unsupervised per-beam calibration from a moving platform (Levinson & Thrun).

### Cited Findings
- 4-step Spectralon plate (99/50/25/12%) used as the reference for calibrating intensities across measurements; laboratory correction transferable to the field because range and angle effects do not interact — [Krooks et al., ISPRS Annals II-5/W2 2013](https://isprs-annals.copernicus.org/articles/II-5-W2/145/2013/isprsannals-II-5-W2-145-2013.pdf).
- Kaasalainen et al. (2011) correction table: Spectralon 99%, tarps, sand, gabbro over several distances and angles — [TUNI](https://researchportal.tuni.fi/en/publications/analysis-of-incidence-angle-and-distance-effects-on-terrestrial-l/); [doi:10.3390/rs3102207](https://0-dx-doi-org.brum.beds.ac.uk/10.3390/rs3102207).
- Tan & Cheng (2015): polynomial f₁(ρ)·f₂(cos α)·f₃(R) fitted by least squares on known-reflectance diffuse targets; a Faro variant uses linear interpolation in a target look-up table — [SPIE](https://biomedicaloptics.spiedigitallibrary.org/journals/journal-of-applied-remote-sensing/volume-9/issue-01/094094/Intensity-data-correction-based-on-incidence-angle-and-distance-for/10.1117/1.JRS.9.094094.full); [ECNU](https://pure.ecnu.edu.cn/en/publications/correction-of-incidence-angle-and-distance-effects-on-tls-intensi/).
- Höfle & Pfeifer (2007) data-driven: least-squares fit of a global correction function over homogeneous areas, with no reference targets — [Heidelberg](https://heibib.ub.uni-heidelberg.de/search/Record/1885638124).
- Xu et al. (2017): overlap-driven adjustment of the Oren–Nayar roughness parameter — [MDPI](https://www.mdpi.com/2072-4292/9/11/1090).
- Tan & Cheng (2020, GRSL): the distance function is estimated from naturally homogeneous surfaces, target-free — [ECNU](https://pure.ecnu.edu.cn/en/publications/distance-effect-correction-on-tls-intensity-data-using-naturally-/).
- Laasch, Medic & Wieser (2025): automatic in-situ calibration for TLS (using station overlap) and MLS (little or no overlap; global vs per-segment AOI functions) — [ISPRS Archives 2025](https://isprs-archives.copernicus.org/articles/XLVIII-G-2025/845/2025/); [TU Wien](https://repositum.tuwien.at/handle/20.500.12708/218316?mode=full).
- Levinson & Thrun, "Unsupervised Calibration for Multi-beam Lasers", ISER 2010 (Springer STAR vol. 79, 2014). On a Velodyne HDL-64E each beam has a non-linear remittance error relative to the world and to the other beams. The method aggregates points over many poses and assumes points lie on contiguous surfaces seen by several beams. It minimizes an energy that makes the beams agree, building a per-beam response curve (raw → calibrated remittance). It needs accurate localization (a SLAM/IMU trajectory) — [CMU RISS poster](https://riss.ri.cmu.edu/education/colby-poster.pdf); [Levinson thesis, Stanford 2011](https://purl.stanford.edu/zx701jr9713); [ISPRS Annals II-5/W2-85 2013](https://isprs-annals.copernicus.org/articles/II-5-W2/85/2013/isprsannals-II-5-W2-85-2013.pdf); [US patent 12135395](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12135395).
- Industry practice for automotive sensors: per-channel, per-power-level linear (gain + bias) model from known-reflectance targets at several distances and orientations — [US 11067676](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11067676); a unit-specific reflectance inference model from range and raw intensity — [US 10598791](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10598791).

### Inferences
- **Recommended target protocol** (synthesis of the above):
  - Spectralon or calibrated panels at about 5–10%, 20–25%, 50% and 90–99%, large enough to fill the footprint, with margin for edge rejection.
  - Ranges: dense in the near zone (0.5, 0.75, 1, 1.5, 2, 3, 4, 6, 8 m), sparser beyond (10, 15, 20, 30 m).
  - Angles: 0–70° in 10° steps at a few ranges, enough to verify separability.
  - For a multi-beam sensor, every beam must see the panels (tilt the sensor or move the panels).
  - Fit log I = log f_b(R) + log g(α) + log h(ρ) per beam b. With a log-amplifier sensor the model is additive in log space, which is convenient.
  - High-reflectance targets near range test saturation; low-reflectance targets far away test the noise floor.
- **Relative self-calibration for a SLAM pipeline** (cave survey without targets):
  - Use the mapped trajectory to collect, for each map voxel, observations from many ranges, angles and beams.
  - Solve jointly for per-voxel albedo ρ_v, per-beam f_b(R) (a spline in R) and a global or per-segment g(α), by alternating least squares (Levinson-style energy + Höfle data-driven).
  - Fix the scale ambiguity with one reference panel or by normalizing the median albedo.
  - This yields Level 2 (consistent pseudo-reflectance). Absolute reflectance needs at least one known target.
- **Sensor modes:** sensors that already output "calibrated reflectivity" (e.g. Ouster's reflectivity channel) may have range compensation built in. Verify on a constant target before stacking another 1/R² correction (double-correction risk).

### Gaps
- Spectralon reflectance-certification uncertainty at 905 nm (typically ±1–2% for 99% panels per vendor data) was not verified in this session.
- Exact sampling grids used in Kaasalainen 2011 or Tan & Cheng 2015 were not retrieved.

## 5. Residual uncertainty after correction, and separable material classes

### Takeaway
Good published corrections bring intensity to within roughly 3–10% of reflectance:
- <3% vs spectrometer for a Riegl VZ-400i with piecewise + Oren–Nayar (Xu 2017);
- "below the ~10% noise level" for ALS (Höfle & Pfeifer 2007);
- CV reductions of 50–93% on homogeneous targets (Tan & Cheng).

For a 905 nm mobile ToF sensor with per-beam differences and near-range effects, 5–15% relative residual is a realistic expectation. That supports perhaps 3–6 broad brightness classes (e.g. dark wet rock / clay / limestone / calcite-flowstone / retroreflector), not fine lithology.

### Cited Findings
- Xu et al. (2017): <3% difference from spectrometer reflectance after correction (Riegl VZ-400i) — [MDPI](https://www.mdpi.com/2072-4292/9/11/1090).
- Höfle & Pfeifer (2007): both methods reduce disturbing effects below a noise level of about 10% — [Heidelberg dissertation](https://archiv.ub.uni-heidelberg.de/volltextserver/36930/1/hoefle_dissertation_2007.pdf).
- Tan et al. (2016): corrected/raw CV ratios of about 0.19 and about 0.07 for two Faro scanners — [ECNU](https://pure.ecnu.edu.cn/en/publications/intensity-data-correction-for-the-distance-effect-in-terrestrial-/).
- Tan & Cheng (2020): CV −52%; a VZ-4000 intertidal study: CV about −54% — [ECNU](https://pure.ecnu.edu.cn/en/publications/distance-effect-correction-on-tls-intensity-data-using-naturally-/).
- Carrea et al. (2016) used Oren–Nayar-corrected intensity for lithological differentiation on rock faces; the number of classes and accuracy were not retrieved — [UA record](https://observatorio-cientifico.ua.es/documentos/652b2ab2ebee473633a3aa5a).
- Kashani et al. (2015) note the lack of standards; comparisons across papers are hindered by different terminology and metrics (CV, percent reflectance, classification accuracy) — [PMC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4701271/).

### Inferences
- **Rule of thumb for class separability:** classes are separable if their mean reflectance difference exceeds about 2–3× the combined residual σ.
  - With a 5% (absolute) residual σ on a 0–100% scale: about 6–8 levels in principle.
  - With a realistic 10–15% *relative* residual, plus within-material natural variability (wetness can halve rock reflectance at NIR), expect 3–5 robust classes from intensity alone.
  - Adding geometric features (roughness, curvature) or multi-echo statistics increases this.
- **Wetness:** water strongly darkens most minerals at 905 nm and adds a specular term. In caves it is likely the single largest "material" confounder, larger than residual radiometric error.

### Gaps
- No published material-classification accuracy for intensity-only classification with a 905 nm automotive-class sensor was found in this session.
- Carrea et al.'s class count and accuracy were not retrieved.

## 6. Multi-echo, beam footprint, partial hits ("mixed pixels")

### Takeaway
When the footprint straddles an edge, the energy splits between surfaces. The intensity of each echo (or of the single merged echo) is then biased low, because only part of the cross-section is hit, and the range is a blend: mixed pixels at depth discontinuities. These points must be detected and excluded from calibration and classification. Typical detectors: large range jumps between neighbours, multiple returns, low intensity relative to neighbours.

### Cited Findings
- An edge where the footprint straddles two surfaces gives a blended return. The footprint elongates from a circle to an ellipse with incidence angle. Mixed pixels fall into three types: between foreground and background, in front of the foreground, behind the background — [arXiv 2503.07472 (review, 3D concrete printing inspection)](https://arxiv.org/pdf/2503.07472).
- The "edge effect" is described as one of the most serious problems for TLS intensity and must be corrected before analysis. An intensity-recovery algorithm corrected about 35.71% of edge-affected points — [NTU thesis record](https://tdr.lib.ntu.edu.tw/handle/123456789/68228?mode=full).
- Partial beam blockage: the distance reading depends more on the blocked radiant power than on the blocked area, with an influence zone of about 1.5–2.5 m around a foreground object — [Sensors, Mar 2011 (DOAJ)](https://doaj.org/article/c487fe9db6664f2f9a16798607ca6c21).
- Tang, Huber & Akinci compared depth-discontinuity / mixed-pixel detection algorithms and quantified edge loss in laser-scanned data — cited in [Remote Sens. 12:2855](https://mdpi-res.com/d_attachment/remotesensing/remotesensing-12-02855/article_deploy/remotesensing-12-02855.pdf).
- Hyperspectral LiDAR work treats incidence angle, range and sub-footprint effects together, and radiometric correction reduces the discrepancies — [Sensors 21(9):2960](https://www2.mdpi.com/1424-8220/21/9/2960).
- Höfle & Pfeifer (2007) and Jutzi & Gross (2009) include beam divergence in the normalization (the footprint area ∝ R²β_t²). Target-size dependence is what separates extended (1/R²) from point (1/R⁴) behaviour — [Heidelberg](https://heibib.ub.uni-heidelberg.de/search/Record/1885638124); [ISPRS 2009](https://www.isprs.org/proceedings/XXXVIII/3-W8/papers/213_laserscanning09.pdf).

### Inferences
- **Energy split:** with fill fraction f of the footprint on surface 1, echo 1 carries about f·ρ₁ and echo 2 (if resolved) about (1−f)·ρ₂·(R₁/R₂)².
  - Range resolution: echoes are separated only if ΔR exceeds about c·τ/2 (about 0.5–1 m for 3–5 ns pulses, sensor-dependent).
  - Closer echoes merge into one return with a blended range and a biased intensity.
- **Multi-echo sensors:** the "strongest" vs "last" return mode affects which surface's intensity is reported. Only single-return or clearly isolated last returns with full footprint should feed the calibration.
- **Practical filters for calibration and classification:**
  - reject points whose range differs from neighbours by more than a few footprint widths;
  - reject points with a second echo;
  - reject points within 1–2 footprint diameters of a depth edge;
  - reject points with α > 70–75°;
  - reject points in the saturation and near-range zones not covered by the fitted f(R).
- **Footprint size:** for spinning 905 nm sensors with about 2–3 mrad divergence (sensor-specific, not verified here), the footprint at 20 m is 4–6 cm. In narrow cave passages with stalactites and flowstone, a sizeable share of points may be edge-affected.

### Gaps
- No quantitative study of mixed-pixel intensity bias for 905 nm automotive multi-beam sensors was found.
- Specific beam divergence and pulse-width figures for the sensor used by dMapper were not researched here.
