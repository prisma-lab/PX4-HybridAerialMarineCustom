# hybridFirmware1.15merge

Merge di `tesi_paola` nella base `firmware1.15mods`, con priorità al lavoro originale di Andrea.

- Primo parent: `9747009bc09f27500c937760e18efd88e3544f4d` (`firmware1.15mods`).
- Secondo parent: `de92ad0c34c35dd1b413b60d24303688fdc44b4c` (`tesi_paola`).
- Antenato comune: `2ae652a202dfd257d26e4bab9f94ab173ef9fe25`.

Il modulo `marine_navigation`, le missioni ibride, i relativi parametri, il mixer, il modello `USV_test` e l'airframe 4010 conservano il contenuto della base. Nei file condivisi sono aggiunti i raccordi per `marine_takeoff`: registrazione del modulo, messaggi, DDS, selezione della modalità e comportamento specifico del decollo marino. Le modifiche della tesista che avrebbero sostituito il comportamento delle modalità esistenti non sono applicate.

Eccezioni autorizzate dopo il merge: nella modalità Marine FF i due assi vengono aggiornati indipendentemente. Il precedente `else`, legato al solo pitch, lasciava il comando sinistro al valore precedente con pitch centrato e poteva annullare la zona morta del throttle. Nella modalità TS il comando differenziale segue l'equazione richiesta da Andrea, con fattore `2 * throttle` anziché `throttle + 1`. Missioni, mixer, parametri e altre modalità rimangono invariati.

## Identificativi

| Identificativo | Branch integrato | Motivo |
| --- | --- | --- |
| `NAVIGATION_STATE_MARINE_TAKEOFF` | 8 | Slot precedentemente libero; preserva External 8 e tutti gli stati PRISMA originali. |
| `PX4_CUSTOM_MAIN_MODE_PRISMA` | 11 | Identico alla base. |
| `PX4_CUSTOM_SUB_MODE_MARINE_TAKEOFF` | 5 | I sotto-stati PRISMA originali 1–4 restano invariati. |
| Selezione RC / enum eventi `marine_takeoff` | 39 | Conserva la selezione della tesista; non coincide con il valore del nav_state. |

Client ROS/QGC o script che usavano i valori del branch `tesi_paola` devono usare nav_state 8 oppure main/sub-mode 11/5. Le integrazioni esistenti di Andrea mantengono i loro valori.

Sono aggiunti `MarineSync`, `MarineWave`, `MarineWaveResult`, `MarineWavePrediction` e `MarineWindow` e il flag `flag_control_marine_takeoff_enabled`. I topic DDS preesistenti sono conservati. Rigenerare/allineare `px4_msgs` e i client DDS alle definizioni di questo branch prima di usare ROS: l'aggiunta di un campo a `VehicleControlMode` cambia lo schema del messaggio.

Le deroghe su home position, auto-disarm e controlli di arming del nuovo modo seguono la tesi. Il rilassamento del rilevamento di movimento e rotazione è attivo soltanto con `flag_control_marine_takeoff_enabled`, preservando il rilevamento originale nelle altre modalità. Le regole failsafe e le tre modalità esterne della base non sono sostituite.

## Simulazione opzionale

Il modello originale e i suoi parametri rimangono invariati. La variante `USV_takeoff` usa la geometria e la configurazione della base, aggiungendo il plugin di misura delle onde `libCentralSurfacePlace.so` della tesi. Le mesh sono condivise con `USV_test`; i topic dei thruster della variante usano `USV_takeoff_0`.

L'airframe `4011_gz_USV_takeoff` usa il mondo `WaterTakeOffPaola`, che conserva i parametri delle onde della tesi. Richiede Gazebo e i plugin VRX esterni già richiesti dai modelli. Con tali dipendenze disponibili:

```sh
make px4_sitl_default gz_USV_takeoff
```

La selezione della nuova modalità dalla console è `commander mode marine_takeoff` (con veicolo disarmato e appoggiato). Gli algoritmi FFT/ZOH del modulo della tesista sono importati; non è stata eseguita una prova di decollo completa con onde simulate o hardware.

## Verifiche eseguite

```sh
source /workspace/px4-venv/bin/activate
make px4_sitl_default -j5
make tests TESTFILTER='.*' -j5
python3 Tools/test_hybrid_merge.py
python3 Tools/test_marine_mode_fixes.py
```

- Compilazione SITL riuscita, inclusi `marine_navigation`, `marine_takeoff`, KissFFT e ZOH.
- Test di regressione: 30 file originali verificati, ammettendo soltanto le correzioni FF e TS autorizzate; 15.360 confronti dei flag di controllo per le modalità esistenti; identificativi, codici MAVLink e nomi originali conservati.
- Runtime con il target `px4_sitl_test` senza lockstep: entrambi i moduli avviati contemporaneamente, `marine_navigation` a 50 Hz e `marine_takeoff` a 250 Hz in stato inattivo; arresto regolare verificato. Nel target default il clock lockstep richiede un simulatore per avanzare.
- XML del nuovo modello/mondo e YAML DDS validi; nessuna registrazione DDS duplicata.
- Suite CTest: **134/137 passati**, tre fallimenti diagnosticati e lasciati visibili:
  - `unit-MessageDisplay`: il test originale confronta una stringa copiata senza terminatore nullo.
  - `functional-ModeManagement`: il test originale usa l'indice 3, ma la base riserva solo tre modalità esterne (indici 0–2). Implementazione, test e identificativi esterni restano quelli della base.
  - `sitl-mavlink`: i casi negativi su `/bogus` ricevono EROFS sul filesystem cloud invece dell'ENOENT atteso.

Gazebo, scambio DDS con ROS, decollo attivo e hardware non sono stati verificati in questo ambiente. Le verifiche di compilazione e regressione non sostituiscono queste prove.

## Correzioni successive del controllo marino

- Marine FF aggiorna sempre entrambi i comandi, ciascuno con la propria zona morta di 0.09. Il propulsore sinistro non dipende più dall'aggiornamento del pitch.
- Marine TS usa `T_L = j_t` per steering non negativo e `T_L = j_t * (1 + 2*j_s)` per steering negativo; `T_R = j_t * (1 - 2*j_s)` per steering positivo e `T_R = j_t` per steering non positivo. La zona morta 0.09 su throttle e steering resta invariata. Con throttle zero entrambi i comandi sono zero; a steering +0.5 il destro si ferma, a +1 inverte il segno (simmetricamente per il sinistro a steering negativo). Il test verifica 81 combinazioni, inclusi reverse e confini della zona morta.
- Marine Takeoff scrive accelerazione, roll e pitch nei rispettivi buffer. In precedenza il roll finiva nel buffer della ground truth, il pitch nel buffer del roll e il buffer del pitch non veniva alimentato.
- Uscendo da Marine Takeoff viene azzerata anche un'acquisizione incompleta, senza attendere che il decollo sia già programmato. Sono cancellati deadline, finestre, campioni e stato ZOH; il contatore FFT è un membro dell'istanza che viene reinizializzato. Il modulo termina il ciclo subito quando la modalità è inattiva e usa una sola lettura dei flag per ciclo.
- Il test `Tools/test_marine_mode_fixes.py` esegue i frammenti di produzione con l'implementazione ZOH reale: 36 combinazioni degli assi FF, aggiornamenti con pitch centrato, associazione dei buffer, uscita durante acquisizione, cancellazione di un decollo pendente e rientro nella modalità.

### Marine mode admission

Marine Manual, Marine Manual TS, Marine Manual FF and Marine Takeoff share
one admission rule in `UserModeIntention`: entry from any other mode requires
`vehicle_land_detected.landed` and a disarmed vehicle. Missing landing data
rejects entry. Transitions within this group have no landing/disarming
constraint; ordinary health and failsafe checks remain active. Leaving this
group does not impose a new landing/disarming constraint.

This intentionally replaces the previous `maybe_landed` test and removes
the Auto Marine source exemption. CLI, MAVLink, RC and mode-executor requests
use the same rule, including forced requests and replacement modes.

Regression: `python3 Tools/test_marine_mode_admission.py`.

### Spectral prediction aligned with thesis chapter 7

Future vertical displacement is reconstructed directly from the combined
acceleration spectrum, multiplying each selected harmonic by `-1/omega^2`.
The intermediate future-acceleration reconstruction, FFT and IFFT are removed.
Roll and pitch retain their signed DC term (`FFT[0].real / FFT_SIZE`); only
acceleration DC is excluded from the integrated prediction. Existing spectral
selection, acquisition, window duration and command timing remain unchanged.

Regression: `python3 Tools/test_marine_prediction.py` compiles the production
functions with KISS FFT and checks analytical heave, acceleration DC removal,
attitude bias and oscillation preservation, window rejection for tilted craft,
and numerical agreement with the former integration route.
