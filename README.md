# NB-IoT over NTN (Thesis Simulator)

Thesis simulator combining two ns-3 extensions to model **NB-IoT over
Non-Terrestrial Networks**:

- [**ns-3-ntn**](https://gitlab.com/mattiasandri/ns-3-ntn) — NTN extension
  (dev-build fork of ns-3.36.1) implementing 3GPP TR 38.811 / TR 38.821
  channel and geometry for satellite links.
- [**LENA-NB**](https://github.com/tudo-cni/ns3-lena-nb) — LTE/NB-IoT
  extension for ns-3 (originally targeting ns-3.32), providing Narrowband
  and NB-IoT PHY/MAC functionality.
- **Winner+** propagation model integrated from an external repository for
  more realistic terrestrial/satellite radio propagation.

## Run a scenario

```bash
cd ns-3-ntn
./ns3 configure --enable-examples
./ns3 build
./ns3 run "scratch/lena-nb-5G-ntn-scenario.cc"
```

Output files are written to the `logs/` folder.

## References

- ns-3: <https://www.nsnam.org>
- ns-3-ntn: <https://gitlab.com/mattiasandri/ns-3-ntn>
- LENA-NB: <https://github.com/tudo-cni/ns3-lena-nb>
