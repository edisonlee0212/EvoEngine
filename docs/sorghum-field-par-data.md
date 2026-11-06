# Sorghum field ceptometer PAR data

Source files: `D:\SorghumFieldsData\Ceptometer Protocol.docx`, `Ceptometer.pptx`, and `taX_cal.csv`, `taY_cal.csv`, `tbX_cal.csv`, `tbY_cal.csv` in the same folder.

Each CSV row has a timestamp and three calibrated photosynthetically active radiation (PAR) readings in `µmol/m²/s`: below the canopy, at the canopy, and above the canopy. The loggers sampled at 1 Hz. These are instantaneous photon flux densities, not daily light integrals. `A` and `B` identify the two three-sensor logging stations; `X` and `Y` identify alternate field positions:

| CSV group | Plot furrow | Genotype of both adjacent plots |
| --- | --- | --- |
| A-X | 7407/7408 | Pawaga |
| B-X | 7403/7404 | BTx623 |
| A-Y | 7415/7416 | Pawaga |
| B-Y | 7411/7412 | BTx623 |

The genotype assignments come from the matching plot IDs in `D:\SorghumFieldsData\2021-07-15__18-59-42-459_sorghum_plant_reports.tar`: its plant report directories are named `Pawaga_7407_...`, `Pawaga_7408_...`, `Pawaga_7415_...`, `Pawaga_7416_...`, `BTx623_7403_...`, `BTx623_7404_...`, `BTx623_7411_...`, and `BTx623_7412_...`. This identifies the plots in the July 15 plant scan; the ceptometers themselves were mounted in the furrows between each plot pair.

For example, averaging all 300 one-second samples from 2021-07-15 12:00:00 through 12:04:59 gives these PAR values in `µmol/m²/s`:

| Furrow | Below canopy | At canopy | Above canopy |
| --- | ---: | ---: | ---: |
| A-Y (Pawaga 7415/7416) | 269.579 | 510.583 | 878.365 |
| B-Y (BTx623 7411/7412) | 165.695 | 444.781 | 842.880 |

These are five-minute examples, not date-wide means. Match the full time series by timestamp when validating simulated PAR; use above-canopy PAR as the incident-light reference and the lower sensors as validation targets after checking channel labels and calibration.

The protocol says the sensors are horizontal, with their long axes 45° to the planting rows. The three mounting poles increase in height from south to north. The stations were moved between X and Y positions, generally twice per week; the timestamp and file suffix together identify where a reading was taken. The first and last days of each continuous CSV segment contain only part of a day.

The actual timestamps in the CSV files span these segments (all dates in 2021). The files do not state a timezone:

| File | Rows | Date segments with data |
| --- | ---: | --- |
| `taX_cal.csv` | 3,171,445 | Jun 29–30; Jul 2–7, 9–15, 16–21, 26–28; Aug 2–4, 11 (6 min), 25–Sep 3; Sep 9–17 |
| `tbX_cal.csv` | 3,879,600 | Jun 29–30; Jul 2–7, 9–15, 16–21, 26–28, 30–Aug 4; Aug 11–18, 25–Sep 3; Sep 11–17 |
| `taY_cal.csv` | 3,168,283 | Jun 30–Jul 2; Jul 7–9, 15–16, 21–26, 30 (3 h); Aug 4–11, 18–25; Sep 3–9, 17–23 |
| `tbY_cal.csv` | 3,321,422 | Jun 30–Jul 2; Jul 7–9, 15–16, 21–26, 28–30; Aug 4–11, 18–25; Sep 3–9, 17–23 |

The July 23 presentation documents seven placement periods from Jun 29 through Jul 21. The CSVs contain additional later periods through Sep 23. The presentation's separate RGB/laser scanner dates are not PAR measurement dates.

The presentation flags several data-quality issues: B2 and B3 were captured under each other's dataset field labels; calibration errors are visible; the mounting poles cast shadows; and diffuser deformation changes with temperature. The `_cal` filenames indicate calibrated values, but the presentation says that calibration should be improved using the raw measurements. Verify the B channel-to-height mapping and re-calibration before treating these values as ground truth for light simulation. The documents do not give precise sensor heights. For field comparison, align the same plot, timestamp, canopy height, and light condition; calculate canopy transmission only after that correction. The engine's RGB illumination result also needs a calibrated spectral conversion before direct comparison with PAR photon flux.
