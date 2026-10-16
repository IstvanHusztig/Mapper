# Near-infrared (~905 nm) reflectance of geological materials and LiDAR-intensity discrimination in caves, mines and outcrops

Scope note for the report writer: the USGS (crustal.usgs.gov, sciencebase.gov) and JPL ECOSTRESS (speclib.jpl.nasa.gov) spectral-library servers could not be reached from this research session (the network proxy refused them), so **no library reflectance value at 905 nm was read directly from splib07, ECOSTRESS or RELAB**. Numeric values below are either cited from papers found in search, or are clearly marked as unverified background knowledge in the Inferences sections. Several key full texts (Franceschi 2009, Burton 2011, Nováková 2022, Carrea 2016) were only available as abstracts or bibliographic records; details beyond the abstracts are flagged.

Wavelength reminder: many published TLS geology studies used **1535/1550 nm** (Optech ILRIS) or **1550 nm** (RIEGL VZ series), not 905 nm. Where that is the case it is stated, because water and clay behave very differently at 1550 nm (strong water absorption) than at 905 nm (weak water absorption).

## 1. Typical reflectance at ~905 nm of rocks, minerals and surface states

### Takeaway
At 905 nm the main material-specific absorber among common cave/outcrop materials is **ferric iron (Fe3+)**: hematite and goethite have a crystal-field absorption band centred right at 850–1020 nm, so iron-stained rock, red clays and ochre look darker to a 905 nm LiDAR than their visible colour suggests. Pure carbonates, gypsum, calcite speleothems and snow have no absorption feature near 905 nm and are bright. Coal and organic-rich deposits are uniformly dark. Liquid water absorbs only weakly at 905 nm (about 0.07 cm⁻¹), so wetting darkens rock mainly through refractive-index (optical) effects, not by water absorption as it does at 1550 nm. Reliable per-material numbers at 905 nm from splib07/ECOSTRESS could not be fetched in this session.

### Cited Findings
**Iron oxides (the most important 905 nm absorber)**
- In 176 Fe-oxide samples the lowest-energy crystal-field band (⁶A₁→⁴T₁ transition) falls at **848–906 nm for hematite** and **909–1022 nm for goethite** and other Fe oxides; the electron-pair band is at 521–565 nm (hematite) vs 479–499 nm (others) — [Scheinost et al. 1998, Clays and Clay Minerals 46(5), doi:10.1346/CCMN.1998.0460506](https://www.doi.org/10.1346/CCMN.1998.0460506)
- Room-temperature band centres of the lowest-energy crystal-field band are about **0.86 µm hematite, 0.91 µm goethite, 0.92 µm maghemite, 0.98 µm lepidocrocite** — [Sherman & Waite 1985, American Mineralogist 70, 1262](https://msaweb.org/AmMin/AM70/AM70_1262.pdf)
- Samples containing hematite and goethite show a reflectance minimum near 900 nm whose position depends on the hematite/goethite proportion — [Cambridge Core record (Clays and Clay Minerals)](https://www.cambridge.org/core/product/4A5D2C8E315B8B5DEDA8BBF495944944)
- The goethite/hematite ratio can be predicted from second-derivative bands; detection limit <5 g/kg, about 10× better than routine XRD — [Scheinost et al. 1998](https://www.doi.org/10.1346/CCMN.1998.0460506)
- In iron-ore mining, the width (FWHM) of the 900 nm ⁶A₁→⁴T₁ feature in goethite-dominated zones relates to goethite type (ochreous vs vitreous) — [MDPI Minerals 12(3):381](https://www.mdpi.com/2075-163X/12/3/381/html)
- Band position shifts slightly with particle size (wavelength-dependent scattering) — [Scheinost et al. 1998](https://www.doi.org/10.1346/CCMN.1998.0460506)

**Carbonates, gypsum, clays**
- Carbonate absorption features are in the SWIR at 2320–2350 nm (calcite 2335 nm; dolomite shifted to shorter wavelengths); this is how hyperspectral imaging separates limestone from dolomite — i.e. the diagnostic carbonate features are far from 905 nm — [Kurz et al., ISPRS Archives XXXIX-B5 (2012)](https://isprs-archives.copernicus.org/articles/XXXIX-B5/417/2012/isprsarchives-XXXIX-B5-417-2012.pdf); [Kurz et al. EGU2010-7777](https://meetingorganizer.copernicus.org/EGU2010/EGU2010-7777.pdf)
- Reflectance differences between limestones, marls and clays at 1535 nm were attributed to the beam's interaction with water-bearing (clay) minerals — [Franceschi et al. 2009, ISPRS J. 64(6):522–528, doi:10.1016/j.isprsjprs.2009.03.003](https://www.earth-prints.org/entities/publication/bb9c4bdf-3743-4fbd-a5b6-db80cfd4cd4d)

**Granite, basalt, sandstone**
- ASTER Spectral Library v1.2 (Baldridge et al. 2009) contains reflectance spectra for granite, basalt and sandstone (and snow) — values at 905 nm were not visible in the snippet — [ResearchGate figure citing ASTER v1.2](https://www.researchgate.net/figure/Spectral-reflectance-data-for-snow-and-rock-granite-basalt-and-sandstone-from-the_fig1_305788817)
- Hovis & Callahan (1966) measured total reflectance of igneous rocks, tuffs and a red (iron-oxide-rich) sandstone from 0.5 to 22 µm — [JOSA 56(5):639](https://opg.optica.org/josa/abstract.cfm?uri=josa-56-5-639)

**Coal and organic material**
- Anthracite reflectance curves are nearly flat across the VNIR with no obvious absorption valleys; as coal rank decreases, reflectance, NIR slope and number/depth of absorption features increase — [Spectroscopy and Spectral Analysis 39(6):1717 (2019)](https://m.researching.cn/articles/OJ73a3f23ddbd919e4)
- Coal-band albedo at 550 nm tracks vitrinite reflectance (rank) — [AusIMM Mining Geology Conf. 2017, "Hyperspectral signature of coal bands – black over black"](https://www.ausimm.com/publications/conference-proceedings/tenth-international-mining-geology-conference-2017/hyperspectral-signature-of-coal-bands---black-over-black/)
- A NIR dataset paper notes the USGS library mainly holds mineral spectra, so it may not contain coal entries — [Sci. Data / PMC11178847](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11178847/)

**Water, ice, snow**
- Liquid water absorption coefficient (Hale & Querry 1973 table): **≈0.068 cm⁻¹ at 900 nm** and **≈0.45 cm⁻¹ at 970 nm** (the 970 nm water band); the file notes wavelength conversion, so check the original — [Hale & Querry table, Oxford course data](https://users.physics.ox.ac.uk/~pierrehumbert/PrinciplesPlanetaryClimate/Data/WorkbookDatasets/Chapter4Data/AbsorptionData/WaterAbsorptionOrig.txt)
- Kou, Labrie & Chylek (1993, Appl. Opt. 32:3531) give accurate imaginary refractive index of water at 22 °C for 0.65–2.5 µm (numbers not retrieved) — [OMLC water spectra page](https://omlc.org/spectra/water/abs/)
- Snow: reflectance drop of 18 % observed around 960 nm when snow changed from dry to wet; other work centres on the 1030 nm ice band, which shifts with liquid-water content — [Annals of Glaciology, spectral snow-reflectance models for melting snow](https://www.cambridge.org/core/journals/annals-of-glaciology/article/spectral-snowreflectance-models-for-grainsize-and-liquidwater-fraction-in-melting-snow-for-the-solarreflected-spectrum/AF7B8358496D46F1F3A1C0A98962B0BB)
- Reference brightness at 905 nm: white TiO₂ pigments reflect around 60 % at 905 nm (patent context) — [USPTO patent 11795329](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11795329); dark/black materials absorb NIR and markedly reduce LiDAR precision/accuracy — [PMC9610704](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9610704/)
- At 905 nm, LiDAR reflectance follows the same theoretical framework as optical reflectance but is more sensitive to surface roughness and microscopic scattering — [arXiv 2511.09840](https://arxiv.org/pdf/2511.09840)

### Inferences
- **Unverified background values (from general spectroscopy knowledge, NOT read from libraries in this session; must be checked against splib07/ECOSTRESS before use):** pure calcite, gypsum and fresh white flowstone/moonmilk are typically very bright at 900 nm (often 0.6–0.9 bidirectional reflectance for powders/clean surfaces); grey micritic limestone and dolomite moderate to bright (~0.3–0.6); quartz sandstone moderate-bright (~0.3–0.5), iron-stained/red sandstone lower near 900 nm because of the Fe3+ band; shale/mudstone and clayey cave sediment ~0.1–0.3; granite ~0.2–0.4; basalt dark ~0.05–0.15; coal ~0.03–0.08; fine fresh snow ~0.8–0.95 at 905 nm (ice absorption still weak there), coarse/old snow lower; clear bulk ice and water bodies return little diffuse signal (mostly transmitted or specularly reflected). These are ranges to verify, not citations.
- Because 905 nm sits on the hematite (848–906 nm) band and on the short-wavelength side of goethite (909–1022 nm), iron-oxide coatings (red clays, ochre, iron-stained flowstone, terra rossa) should be markedly darker at 905 nm than at 1064 nm, where reflectance recovers. A 905 nm scanner is therefore, unusually, somewhat sensitive to Fe3+ content — useful for detecting iron staining, but it also means "dark" at 905 nm does not imply "dark in visible light" or "wet".
- Water at 905 nm absorbs ~7× less than at 970 nm and orders of magnitude less than at 1450/1550 nm, so a thin water film barely absorbs; wet darkening at 905 nm comes from reduced refractive-index contrast in pores and from specular loss (see section 2).
- Guano and organic deposits: no spectral source found; expected dark-to-moderate depending on mineralised (phosphate, light) vs fresh organic (dark) content — unverified.

### Gaps
- No direct 905 nm value from USGS splib07, ECOSTRESS or RELAB could be retrieved (servers blocked). The report writer should state that numeric library values need a follow-up extraction (e.g. splib07a ASCII spectra for calcite, gypsum, dolomite, hematite, goethite, kaolinite, montmorillonite, basalt, granite, coal; ECOSTRESS rock spectra).
- No spectral data found for moonmilk, cave guano, or cave sediments specifically.
- No absolute ice/snow reflectance value at 905 nm was found in the session; Warren & Brandt (2008) ice optical constants were not retrieved.

## 2. Effect of moisture/wetness, water films, roughness and grain size

### Takeaway
Wetting lowers LiDAR intensity on rock and sediment in every study found; in caves it is the main reason classification fails. At VNIR wavelengths (including 905 nm) reflectance falls exponentially with moisture and **saturates at low moisture**, so 905 nm indicates "wet vs dry" but not *how* wet; 1550 nm keeps responding up to high water contents. Geometry (range, incidence angle) and roughness must be corrected before any moisture or lithology interpretation.

### Cited Findings
- Soil reflectance (400–2500 nm) decreases nonlinearly with moisture, well fitted by an exponential model, similar across soil types when moisture is expressed as degree of saturation; **VNIR saturates at much lower moisture than SWIR**, so longer wavelengths are better for volumetric moisture >~20 % — [Lobell & Asner 2002, SSSAJ 66:722, doi:10.2136/sssaj2002.7220](https://oa.mg/work/10.2136/sssaj2002.7220)
- Across 28 European soils the exponential form holds; the darkening rate depends on organic carbon, clay and silt; SWIR responds up to ~50 % volumetric water while visible wavelengths saturate around 20 % — [Hick et al. 2026, Eur. J. Soil Sci.](https://eprints.whiterose.ac.uk/id/eprint/242589/)
- In a cave (Gouffre Georges), water or wet surfaces biased intensity toward lower values and complicated material discrimination — [Nováková et al. 2022, Remote Sens. Environ., doi:10.1016/j.rse.2022.113210 (HAL record)](https://hal-ifp.archives-ouvertes.fr/GR-3T/insu-03752599v1)
- Sandstone drying experiments (ETH Zurich): TLS intensities can serve as moisture proxies for sandstone water retention; the authors identify suitable wavelengths and mitigation strategies for systematic effects — [Laasch, Medic & Wieser 2023, ISPRS Annals X-1-W1-2023, 567](https://isprs-annals.copernicus.org/articles/X-1-W1-2023/567/2023/)
- Beach sand with a RIEGL VZ-400 (1550 nm): reflectance linearly related to surface moisture from dry to saturated, standard error ~2.6 %; earlier **shorter-wavelength** TLS work found reflectance insensitive to moisture above ~5 % — [EGU2014-2797](https://opac.geologie.ac.at/ais312/dokumente/EGU/EGU2014-2797.pdf)
- Clear effect of target moisture on airborne and terrestrial scanner backscatter; sand/gravel used as reflectance standards, moisture after rain a practical problem — [Kaasalainen et al., EGU2009-6687](https://meetingorganizer.copernicus.org/EGU2009/EGU2009-6687.pdf)
- Laser intensity on similar-material (rock-analogue) models was sensitive to moisture, incidence angle and distance; recommended incidence <20° and range 4–10 m to simplify correction — [PMC9402562 (Sci. Rep./PLoS-type study)](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9402562/)
- Moisture and weathering reduced intensity contrast between sand-rich and shale-rich intervals in outcrop — [Burton et al. 2011, J. Sediment. Res. 81(5)](https://archives.datapages.com/data/sepm/journals/081/081005/339_gsjsedres810339.htm)
- Dual-wavelength: a 905/1550 nm ratio explained 84 % of variance in leaf water content (only in stable conditions) — [Remote Sensing 9(1):8 (Aalto)](https://acris.aalto.fi/ws/portalfiles/portal/11427412/remotesensing_09_00008_v2.pdf); 690/1550 nm normalized ratio vs equivalent water thickness R² = 0.93 — [Junttila et al. 2017, doi:10.3390/rs9010008](https://doi.org/10.3390/rs9010008)
- Most long-range TLS lasers are NIR and can be strongly absorbed by water (mudflat context) — [ESS Open Archive preprint](https://essopenarchive.org/doi/full/10.1002/essoar.10501756.1)
- **Roughness:** Carrea et al. corrected TLS intensity for range, incidence angle and roughness using the Oren–Nayar model, whose simplified form needs one parameter (standard deviation of facet slope angles); tested on boards, plaster and outcrops with Optech ILRIS — [Carrea et al. 2016, ISPRS J. 113:17–29, doi:10.1016/j.isprsjprs.2015.12.004](https://observatorio-cientifico.ua.es/documentos/652b2ab2ebee473633a3aa5a); [EGU2015-10643](https://meetingorganizer.copernicus.org/EGU2015/EGU2015-10643.pdf?pdf)
- Cave-specific correction must account for acquisition geometry, cave-surface micromorphology and cave aerosol, which "similar to fog, considerably attenuates" the beam — [Nováková et al. 2022](https://hal-ifp.archives-ouvertes.fr/GR-3T/insu-03752599v1); [UPJŠ news](https://www.upjs.sk/prirodovedecka-fakulta/en/actuality/laser-scanning-the-key-to-detecting-darkroom-surfaces/)
- Riegl VZ-1000 range effect calibrated on a reference target from 2.5 to 60 m, fitted by piecewise polynomials (2.5–5, 5–15, 15–60 m) — [EGU2020-18742 presentation](https://presentations.copernicus.org/EGU2020/EGU2020-18742_presentation.pdf)
- Review of LiDAR radiometric processing (range, incidence, atmospheric, instrument effects; inconsistent terminology of normalization/correction/calibration) — [Kashani et al. 2015, Sensors 15(11):28099, doi:10.3390/s151128099](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4701271/)
- Specular ice crystals produce enhanced backscatter without increased extinction (atmospheric lidar context, i.e. specular geometry strongly matters for ice) — [Univ. of Reading, lidar retrievals](https://www.met.reading.ac.uk/~swrhgnrj/publications/retrievals_specular.pdf)

### Inferences
- For a 905 nm cave scanner, wet flowstone or wet limestone will appear darker than dry — mainly through refractive-index/optical-darkening and specular loss of a water film (energy reflected away from the sensor except at near-normal incidence, where a film can instead produce a bright specular spike). Water's own absorption at 905 nm (~0.07 cm⁻¹) is negligible for films of mm thickness.
- Because VNIR darkening saturates early, 905 nm intensity is best used as a binary wet/dry indicator after geometric correction; a moisture gradient (seepage halo) may be visible only near the dry end.
- Incidence-angle correction is especially critical in caves, where walls are seen at grazing angles and ranges are short (near-range detector effects, which are scanner-specific, are larger than at outdoor ranges).

### Gaps
- No study found that quantifies wet vs dry rock intensity specifically at 905 nm (most moisture work is at 1550 nm). Full texts of Laasch et al. 2023 (which compares wavelengths) could not be fetched (DNS failure) — it is the best follow-up source for wavelength-dependent moisture sensitivity on sandstone.
- No numeric grain-size effect at 905 nm for rock powders was retrieved.

## 3. Case studies: intensity-based lithology / surface-condition mapping

### Takeaway
Corrected TLS intensity has repeatedly separated **clay-rich from clay-poor** rocks (marl vs limestone; shale vs sandstone) and **contrasting cave lithologies** (marble vs lherzolite, >84 % accuracy), but nearly all outcrop studies used 1535/1550 nm scanners, where the clay–water absorption contributes. Mine studies (coal vs gangue) combine intensity with geometry or multiple wavelengths. Ice caves are routinely split into ice and rock by intensity in practice, but without published accuracy. Hyperspectral+LiDAR fusion (Kurz, Buckley) is the route to mineralogy; single-wavelength intensity alone is not.

### Cited Findings
**Outcrops and quarries**
- Franceschi, Teza, Preto, Pesci, Galgaro & Girardi (2009), Optech ILRIS-3D at **1535 nm**: limestone/marl/clay reflectances differ in lab and field; intensity gives reliable clay-content information on clean outcrops if distance is corrected; reflectance inversely linear with clay abundance (**r = −0.85**) — [Franceschi et al. 2009, doi:10.1016/j.isprsjprs.2009.03.003](https://www.earth-prints.org/entities/publication/bb9c4bdf-3743-4fbd-a5b6-db80cfd4cd4d); [unipd record](https://www.research.unipd.it/handle/11577/2445616)
- Follow-up: TLS intensity converted into intensity logs as lithology proxy for cyclostratigraphy in limestone–marl series — [Padova PhD thesis (Penasa)](https://research.unipd.it/retrieve/e14fb26f-a1eb-3de1-e053-1705fe0ac030/thesis_phd.pdf) (thesis also presents keypoint-based intensity normalization and a CloudCompare toolkit)
- Burton, Dunlap, Wood & Flaig (2011) "Lidar intensity as a remote sensor of rock properties": distance-normalized intensity approximates reflectivity; quartz-rich sandstones more reflective than clay-rich mudstones; in unweathered core intensity falls with clay and rises with quartz+feldspar; weathering/moisture reduced contrast in outcrop — [J. Sediment. Res. 81(5):339](https://archives.datapages.com/data/sepm/journals/081/081005/339_gsjsedres810339.htm) (scanner wavelength not confirmed in this session)
- Carrea et al. (2016): Oren–Nayar-based correction applied to lithological differentiation (Optech ILRIS) — [doi:10.1016/j.isprsjprs.2015.12.004](https://observatorio-cientifico.ua.es/documentos/652b2ab2ebee473633a3aa5a)
- Humair et al.: TLS of folding in a Swiss Jura quarry, including lithology discrimination from signal strength — [VGC-14 presentation](https://wp.unil.ch/risk/files/2014/03/VGC14_Humair_et_al.pdf)
- Brazilian study: rock-type classification in outcrops from TLS intensity using cluster analysis (K-Clouds software) — [AUSJAL repository](https://repositorio.ausjal.org/handle/20.500.12032/58879)
- Kurz, Buckley, Howell et al.: ground-based hyperspectral + LiDAR integration; Pozalagua Quarry (Spain) workflow separating limestone from dolomite and linking signatures to karst, fractures and dolomite types; geometric digital outcrop models alone are of limited use for mineralogical mapping — [Kurz et al. 2012 ISPRS Archives](https://isprs-archives.copernicus.org/articles/XXXIX-B5/417/2012/isprsarchives-XXXIX-B5-417-2012.pdf); [Kurz PhD, Bergen](https://bora.uib.no/handle/1956/4581); [Close-range HS + lidar for geologic outcrop mapping](https://abdn.elsevierpure.com/en/publications/close-range-hyperspectral-and-lidar-data-integration-for-geologic); [NORCE 2022 Alacón dolomitization](https://norceresearch.brage.unit.no/norceresearch-xmlui/handle/11250/3061193)

**Caves**
- Nováková et al. (2022), Gouffre Georges (N. Pyrenees), Riegl TLS: intensity corrected for geometry, micromorphology and cave atmosphere; classification of marble vs (serpentinized) lherzolite reached **>84 % overall accuracy**; wet surfaces biased intensity low — [doi:10.1016/j.rse.2022.113210, HAL](https://hal-ifp.archives-ouvertes.fr/GR-3T/insu-03752599v1); [UPJŠ news](https://www.upjs.sk/prirodovedecka-fakulta/en/?p=49023). Note: the Riegl VZ-1000 used in the related EGU2020 work operates in the NIR (1550 nm class) — wavelength not confirmed for the 2022 paper.
- Nováková et al. (EGU 2026): cave-floor surface types classified by deep-learning semantic segmentation from **geometric** LiDAR features (not intensity) — [EGU26-807](https://meetingorganizer.copernicus.org/EGU26/EGU26-807.html)
- Golgotha Cave (W. Australia): ground LiDAR to measure stalactite widths, lengths and distribution, linked to ceiling topography and fracture-controlled flow (geometry, not intensity) — [ANSTO record](https://apo.ansto.gov.au/items/aeb3b687-b6c9-76a5-e053-150a9d89ded9)

**Ice caves**
- Austrian ice cave, 10-year Riegl monitoring: RiScan Pro differentiates ice mass from cave rock based on the intensity of the reflected signal (no accuracy given) — [xyHt "Lidar and ice"](https://xyht.com/lidarimaging/lidar-and-ice)
- Hundsalm ice cave point cloud (Riegl VZ-2000i) with rock vs snow/firn/ice classification and intensity attribute — public test dataset — [PANGAEA doi:10.1594/PANGAEA.945655](https://doi.pangaea.de/10.1594/PANGAEA.945655)

**Mines**
- Coal vs gangue: LiDAR intensity + range fused into two-channel pseudo-colour images, DenseNet classification, **94.56 %** gangue recognition accuracy — [Laser & Optoelectronics Progress 61(4):0437005 (2024)](https://www.researching.cn/ArticlePdf/m00002/2024/61/4/0437005.pdf)
- Hyperspectral LiDAR coal/rock classification without calibration (waveform entropy, skewness–kurtosis; RF/SVM); dust in deep mines hampers calibration — [Infrared and Laser Engineering 50(10):20200518 (2021)](https://www.researching.cn/articles/OJ105fedf1009bbbaf)
- 91-channel hyperspectral LiDAR for coal/rock; white-board calibration improved accuracy — [ECNU record](https://pure.ecnu.edu.cn/en/publications/a-91-channel-hyperspectral-lidar-for-coalrock-classification/)
- Single-wavelength SLAM LiDAR on rock cores: intensities differ significantly across lithologies, separable by confidence intervals; intensity depends on surface colour and range; also mapped an underground coal mine section — [WVU thesis](https://researchrepository.wvu.edu/etd/11706)

**Multi-wavelength**
- Optech Titan: first commercial multispectral airborne LiDAR at 532, 1064 and 1550 nm; Titan data improved terrain classification vs standard tools (Monterey, CA) — not a rock study — [SPIE 10191, 101910J](https://proceedings.spiedigitallibrary.org/conference-proceedings-of-spie/10191/101910J/Spectral-LiDAR-analysis-for-terrain-classification/10.1117/12.2276658.full)

### Inferences
- **Transfer from 1535/1550 nm to 905 nm:** The marl/limestone result (Franceschi) is attributed partly to water-bearing clay minerals absorbing at 1535 nm. At 905 nm that absorption is much weaker, so clay-rich rocks will still be darker (clay rocks are generally darker and finer-grained, often with Fe and organic matter) but the contrast mechanism differs and is likely smaller; it should not be assumed to transfer quantitatively. The marble/lherzolite contrast (bright carbonate vs dark Fe-Mg silicate/serpentinite) is broad-band and should transfer to 905 nm.
- Ice vs rock in ice caves: at 905 nm ice absorbs less than at 1550 nm, so clear ice will return more (sub-surface volume scattering and range bias) than at 1550 nm; separability may differ from the Riegl (1550 nm) practice.
- Mine coal/gangue results rely on deep learning or extra channels; plain single-wavelength intensity thresholds are likely only reliable for coal (very dark) vs light gangue.

### Gaps
- **No published study found that uses a 905 nm scanner (e.g. Velodyne/Ouster/Livox/Hesai mobile or SLAM LiDAR) for cave lithology or moisture mapping**; all cave/outcrop intensity papers found used 1535/1550 nm TLS. The WVU SLAM-LiDAR thesis may be 905 nm but the wavelength was not confirmed.
- No study found on LiDAR-intensity detection of speleothems vs bedrock, moonmilk, guano or seepage specifically.
- No study found comparing 532/905/1064/1550 nm on the same rock surfaces.
- Full texts of Burton 2011 and Nováková 2022 (exact wavelength, per-class accuracies, correction equations) not retrieved.

## 4. Separable vs ambiguous material pairs at a single wavelength; role of texture/geometry

### Takeaway
A single NIR wavelength gives one scalar (corrected reflectance) per point, so materials separate only if their reflectance distributions don't overlap after geometric correction. Wetness, weathering and iron staining all push intensity down, so "dark" is ambiguous (wet limestone ≈ dry shale ≈ iron-stained sandstone). Published work resolves this by adding geometry/texture (roughness, morphology), range channels, or extra wavelengths/hyperspectral data.

### Cited Findings
- Moisture and weathering reduce the sand-rich/shale-rich contrast in outcrop — [Burton et al. 2011](https://archives.datapages.com/data/sepm/journals/081/081005/339_gsjsedres810339.htm)
- Wet surfaces bias intensity low and complicate material discrimination in caves — [Nováková et al. 2022](https://hal-ifp.archives-ouvertes.fr/GR-3T/insu-03752599v1)
- Reliable clay-content discrimination only in clean outcrop conditions with distance correction — [Franceschi et al. 2009](https://www.earth-prints.org/entities/publication/bb9c4bdf-3743-4fbd-a5b6-db80cfd4cd4d)
- Roughness changes apparent intensity and must be modelled (Oren–Nayar facet-slope parameter) — [Carrea et al. 2016](https://observatorio-cientifico.ua.es/documentos/652b2ab2ebee473633a3aa5a)
- At 905 nm LiDAR reflectance is particularly sensitive to surface roughness and microscopic scattering — [arXiv 2511.09840](https://arxiv.org/pdf/2511.09840)
- Geometric features alone enable deep-learning classification of cave-floor surface types — [EGU26-807](https://meetingorganizer.copernicus.org/EGU26/EGU26-807.html)
- Fusing intensity with range/geometry (pseudo-colour images) gave 94.56 % coal-gangue recognition — [LOP 2024](https://www.researching.cn/ArticlePdf/m00002/2024/61/4/0437005.pdf)
- Ratios of two wavelengths (905/1550, 690/1550) isolate water content independent of brightness — [Remote Sensing 9(1):8](https://acris.aalto.fi/ws/portalfiles/portal/11427412/remotesensing_09_00008_v2.pdf); [Junttila et al. 2017](https://doi.org/10.3390/rs9010008)
- Single-wavelength TLS intensity cannot resolve mineralogy (e.g. limestone vs dolomite); SWIR hyperspectral absorption at 2320–2350 nm is needed — [Kurz et al. 2012](https://isprs-archives.copernicus.org/articles/XXXIX-B5/417/2012/isprsarchives-XXXIX-B5-417-2012.pdf)

### Inferences (proposed separability matrix at 905 nm, reasoning from the findings above; not validated in any single study)
- **Likely separable (large broad-band contrast):** white calcite speleothems/gypsum/snow vs basalt, coal, dark shale; marble vs lherzolite/serpentinite (shown, >84 %); coal vs light gangue; dry clean limestone vs dark clay sediment.
- **Ambiguous:** wet limestone/flowstone vs dry shale/mudstone; iron-stained (hematite/goethite) carbonate or sandstone vs wet carbonate (both darkened, by Fe3+ band vs moisture); limestone vs dolomite (no 905 nm feature); calcite vs gypsum (both bright, features in SWIR); fresh snow vs white moonmilk/flowstone; clear ice vs wet rock (both low diffuse return, specular).
- **Helpful extra features:** local roughness (Oren–Nayar σ, plane-fit residuals) distinguishes smooth flowstone/ice from fractured bedrock and from granular sediment; morphology (stalagmite/stalactite shapes, flat floors of sediment); temporal change (seepage areas change between scans; repeat-scan intensity differencing); range-noise/multi-echo behaviour (ice and water give range bias and weak/specular returns); contextual position (floor vs wall vs ceiling).
- For the dMapper context (presumably a 905 nm mobile scanner with uncalibrated per-unit intensity), any material inference requires at least range and incidence-angle normalization and, ideally, a per-sensor reference-target calibration; raw intensity differences between passages are likely dominated by geometry and moisture rather than lithology.

### Gaps
- No quantitative confusion matrix found for wet-vs-dry carbonate vs shale at any wavelength.
- No study found using roughness + intensity jointly for speleothem vs bedrock classification.
