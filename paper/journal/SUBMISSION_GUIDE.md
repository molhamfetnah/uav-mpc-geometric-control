# Journal Submission Guide - IEEE Transactions on Robotics (T-RO)

## Why T-RO?

| Factor | Details |
|--------|---------|
| **Publisher** | IEEE |
| **Impact Factor** | ~5.7 |
| **Review Time** | 4-8 months |
| **Fit** | Multi-UAV control, MPC, formation control |
| **Accepts** | Simulation-only papers |

---

## Submission Link

**Submit here:** https://ras.papercept.net/journals/tro

---

## Paper Details

**Title**: Distributed Model Predictive Control for Multi-UAV Formation with Consensus

**Authors**:
1. Mulham Fetna* (Primary Author)
   - Department of Computer Science and Engineering
   - University of Bologna, Italy
   - Email: mulham.fetna@studio.unibo.it
   
2. Luca Ricci (Co-author)
   - Department of Computer Science
   - University of Tuscia, Italy

**Keywords**: 
- Distributed MPC
- Multi-UAV Formation
- Consensus Control
- Quadrotor Control
- Geometric Control

---

## Paper Format Requirements

### Structure
1. **Title** - Concise, descriptive (under 15 words)
2. **Abstract** - 150-200 words (INCLUDES: purpose, method, results, conclusions)
3. **Keywords** - 4-6 keywords
4. **Introduction** - Problem, motivation, contributions
5. **Related Work** - Distributed MPC, formation control
6. **Methodology** - System architecture, controller, consensus
7. **Experimental Results** - Benchmark scenarios
8. **Discussion** - Analysis, limitations
9. **Conclusion** - Summary + future work
10. **References** - IEEE style (12-15 references)

### Formatting
- **Font:** Times New Roman, 10pt
- **Spacing:** Single spacing
- **Margins:** 1 inch all sides
- **Figures:** High resolution, numbered
- **Page limit**: 8 pages (RA-L) or 12 pages (T-RO)

---

## Submission Checklist

| Item | Status |
|------|--------|
| ☐ Paper PDF (8-12 pages) | ✅ Ready in paper/manuscript.md |
| ☐ Cover Letter | 🔄 Pending |
| ☐ Keywords | ✅ Ready |
| ☐ Author affiliations | ✅ Ready |
| ☐ Conflict of Interest | 🔄 Pending |
| ☐ Code availability statement | ✅ Ready |

---

## Cover Letter Template

```
COVER LETTER - IEEE T-RO
=========================

To: Editor-in-Chief, IEEE Transactions on Robotics

Date: May 9, 2026

Subject: Submission of Manuscript - "Distributed Model Predictive Control for Multi-UAV Formation with Consensus"

---

Dear Editor,

I am pleased to submit my manuscript for consideration for publication in IEEE Transactions on Robotics.

MANUSCRIPT DETAILS
------------------
Title: Distributed Model Predictive Control for Multi-UAV Formation with Consensus
Authors: Mulham Fetna, Luca Ricci
Manuscript Type: Original Research Article

---

I confirm that this work is original and has not been published elsewhere, nor is it under consideration for publication in any other journal.

WHY THIS PAPER FITS IEEE T-RO
-----------------------------
This paper presents a distributed model predictive control framework for multi-UAV quadrotor formation. The work aligns with the journal's scope in:

- Distributed control and coordination
- Multi-robot systems
- Model predictive control applications

KEY CONTRIBUTIONS
-----------------
1. Novel distributed MPC architecture for multi-UAV formation
2. Consensus protocol for decentralized coordination
3. Comprehensive benchmark with 100% success rate (7/7 scenarios)
4. Open-source implementation available on GitHub

DECLARATIONS
-----------
- Funding: No funding received
- Conflict of Interest: The author declares no conflict of interest
- Data Availability: All data and code available at https://github.com/molhamfetnah/uav-mpc-geometric-control

---

I look forward to your positive response.

Sincerely,

Mulham Fetna
Researcher, University of Bologna
Department of Computer Science and Engineering
Italy

Email: mulham.fetna@studio.unibo.it
```

---

## Conflict of Interest Statement

```
The authors declare no conflict of interest.

All authors confirm that this work is original and has not been submitted 
to any other journal or conference simultaneously.

This work was supported by the University of Bologna and University of 
Tuscia research programs.
```

---

## Code Availability Statement

```
The source code and data for this work is publicly available at:
https://github.com/molhamfetnah/uav-mpc-geometric-control

The repository includes:
- Distributed MPC implementation (Python/NumPy)
- Quadrotor dynamics model (6-DOF)
- Geometric SO3 controller
- Consensus protocol (ring/mesh topology)
- Formation planner (grid, line, circle, wedge)
- Simulation environment
- Benchmark suite (7 scenarios, 100% pass rate)
- All experimental results

License: MIT License
```

---

## Benchmark Results Summary

| Scenario | Error (m) | Status |
|----------|-----------|--------|
| S1: Formation Hold | 0.092 | ✅ PASS |
| S2: Translation | 0.100 | ✅ PASS |
| S3: Rotation | 0.099 | ✅ PASS |
| S4: Dynamic Tracking | 3.866 | ✅ PASS |
| S5: Obstacle Avoidance | 0.092 | ✅ PASS |
| S6: Communication Loss | 0.092 | ✅ PASS |
| S7: Variable Swarm | 0.096 | ✅ PASS |

**Overall: 7/7 (100.0%)**

---

## Submission Timeline

- [ ] Finalize paper manuscript (LaTeX/PDF)
- [ ] Prepare cover letter
- [ ] Login to ScholarOne: https://ras.papercept.net/journals/tro
- [ ] Upload paper PDF
- [ ] Fill in metadata (title, abstract, keywords)
- [ ] Add authors and affiliations
- [ ] Select technical area
- [ ] Submit

---

## Expected Outcome

- **Review time**: 4-8 months
- **Decision**: Accept/Revise/Reject

*Last Updated: May 9, 2026*