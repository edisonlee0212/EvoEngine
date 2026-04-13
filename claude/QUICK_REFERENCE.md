# Quick Reference - EvoEngine PAR Calculation

## 🚀 After Recompile - Quick Start

### 1. Rebuild Python bindings
```cmd
:: Open "x64 Native Tools Command Prompt for VS 2022"
cd C:\Users\Brenda\code\EvoEngine\claude
rebuild_python_bindings.cmd
```

### 2. Run PAR calculation
```bash
cd C:/Users/Brenda/code/EvoEngine/claude
py -3.9 sorghum_single_leaf_daily_par_evoengine.py
```

### 3. Check results
```bash
ls -lh evoengine_par_results/
head evoengine_par_results/hourly_PAR_maricopa_june13_evoengine.csv
```

---

## ✅ Expected Results

- **Peak PAR:** 1800-2200 µmol/m²/s (at noon)
- **Daily total:** 30-50 mol/m²/day
- **Pattern:** Bell curve, 6 AM - 8 PM
- **Files:** CSV + validation plots

---

## 📁 Key Files

| File | Purpose |
|------|---------|
| `SESSION_SUMMARY_PAR_TESTING.md` | Full session summary with troubleshooting |
| `NEXT_STEPS_AFTER_RECOMPILE.md` | Complete workflow and research options |
| `test_evoengine_compatibility.py` | Test API availability |
| `sorghum_single_leaf_daily_par_evoengine.py` | Main PAR calculation script |
| `rebuild_python_bindings.cmd` | Quick rebuild script |

---

## 🐛 Common Issues

| Problem | Solution |
|---------|----------|
| Import error | Use `py -3.9` not `python` |
| "Must run from VS prompt" | Open x64 Native Tools Command Prompt |
| PAR too high/low | Check validation section in results |
| No plots | Check matplotlib installed: `py -3.9 -m pip list` |

---

## 🎯 Quick Validation

```python
# Expected ranges for Maricopa, AZ in June:
Peak PAR:          1800-2200 µmol/m²/s ✓
Daily total:       30-50 mol/m²/day ✓
Morning rise:      6-7 AM ✓
Evening decline:   6-7 PM ✓
Solar elevation:   75-80° at noon ✓
```

---

## 📞 Where to Look for Help

1. `SESSION_SUMMARY_PAR_TESTING.md` - Current status & blocking issues
2. `NEXT_STEPS_AFTER_RECOMPILE.md` - Full workflow & options
3. `TRANSFER_TO_EVOENGINE_MACHINE.md` - Scientific context
4. `README_EVOENGINE_PAR_ADAPTATION.md` - Implementation details

---

**Status:** Ready for recompile → Run script → Validate results! 🎉
