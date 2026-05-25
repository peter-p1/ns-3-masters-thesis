# NB-IoT over NTN (Thesis Simulator)

Thesis simulator combining two NS-3 extensions to model **NB-IoT over
Non-Terrestrial Networks**:

- [**ns-3-ntn**](https://gitlab.com/mattiasandri/ns-3-ntn) — NTN extension
  (dev-build fork of ns-3.36.1) implementing channel and geometry for satellite links.
- [**LENA-NB**](https://github.com/tudo-cni/ns3-lena-nb) — LTE/NB-IoT
  extension for ns-3 (originally targeting ns-3.32), providing Narrowband
  and NB-IoT PHY/MAC functionality.
- **Winner+** propagation model for
  more realistic radio propagation.

Developed on **Ubuntu 24.04 LTS** (using WSL2).

## 1. Build

First build takes ~5–15 minutes.

```bash
cd ns-3-ntn
./ns3 configure
./ns3 build
```

## 2. Quick verification run

Single short simulation (3 UEs, 10 s):

```bash
cd ns-3-ntn
./ns3 run "scratch/lena-nb-5G-ntn-scenario --numUes=3 --simTime=10s"
```

Output files are written to the `logs/` folder.

## References

- ns-3: <https://www.nsnam.org>
- ns-3-ntn: <https://gitlab.com/mattiasandri/ns-3-ntn>
- LENA-NB: <https://github.com/tudo-cni/ns3-lena-nb>
