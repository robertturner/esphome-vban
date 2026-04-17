# esphome-vban

Composants ESPHome pour diffuser et recevoir du VBAN (audio UDP) sur un ESP32.
Deux composants indépendants : `vban_sender` (micro → réseau) et `vban_receiver`
(réseau → speaker). Compatibles unicast, broadcast et multicast.

## Installation

```yaml
external_components:
  - source: github://powange/esphome-vban
    components: [vban_sender, vban_receiver]
    refresh: 0s
```

## `vban_sender`

Capture le flux PCM 16-bit 16 kHz mono d'un microphone ESPHome et l'émet en
paquets VBAN UDP.

```yaml
vban_sender:
  microphone: atom_mic          # ID d'un composant microphone: ESPHome
  target_ip: "192.168.0.255"    # unicast, broadcast de subnet ou multicast
  target_port: 6980
  stream_name: "AtomBureau"     # 16 caractères max
  gain: 1.0                     # amplification logicielle (0.0–N), optionnel
```

Le composant s'abonne au callback du micro et démarre automatiquement la
capture après le setup. Aucune boucle `interval:` n'est nécessaire.

| Paramètre      | Requis | Défaut     | Description |
|----------------|--------|------------|-------------|
| `microphone`   | ✅     | —          | ID du micro ESPHome |
| `target_ip`    | ✅     | —          | Destination (unicast / broadcast / multicast) |
| `target_port`  | ❌     | `6980`     | Port UDP |
| `stream_name`  | ❌     | `AtomEcho` | Nom du flux VBAN (16 chars max) |
| `gain`         | ❌     | `1.0`      | Gain logiciel avec clipping `±32767` |

## `vban_receiver`

Écoute un port UDP, filtre par stream name, et pousse le PCM reçu vers un
speaker ESPHome. Gère automatiquement l'arrêt/redémarrage du microphone pour
libérer le bus I2S pendant la lecture (le `microphone:` est optionnel — à
fournir uniquement si un `vban_sender` utilise le même bus I2S).

```yaml
vban_receiver:
  speaker: atom_speaker         # ID d'un composant speaker: ESPHome
  microphone: atom_mic          # optionnel: mis en pause pendant la lecture
  listen_port: 6980
  stream_name: "TTS1"           # 16 chars max, doit matcher l'émetteur
  idle_timeout_ms: 1500         # délai sans paquet avant arrêt du speaker
```

| Paramètre          | Requis | Défaut | Description |
|--------------------|--------|--------|-------------|
| `speaker`          | ✅     | —      | ID du speaker ESPHome |
| `microphone`       | ❌     | —      | ID du micro à mettre en pause |
| `listen_port`      | ❌     | `6980` | Port UDP d'écoute |
| `stream_name`      | ✅     | —      | Nom du flux à accepter (16 chars max) |
| `idle_timeout_ms`  | ❌     | `1500` | Durée sans paquet avant `stop()` |

## Format audio supporté

Les deux composants supportent exclusivement :

- Sample rate : **16000 Hz**
- Format     : **PCM 16-bit signed**
- Channels   : **1 (mono)**

Le receiver rejette avec un warning les paquets d'un autre format.

## Exemple complet

Voir [examples/atom-echos3r.yaml](examples/atom-echos3r.yaml) pour une config
M5Stack Atom Echo S3R avec sender + receiver + media_player Home Assistant.

## Diagnostics

Chaque composant expose son état via `dump_config` (visible au boot ou à la
connexion d'un client API) :

```
VBAN Sender:
  Target: 192.168.0.255:6980
  Stream name: AtomBureau
  Socket: OK
  Mic started: YES
  Frames sent: 3069

VBAN Receiver:
  Listen port: 6980
  Stream name: TTS1
  Socket: OK
  Packets received: 124
  Packets lost:     0
  Out of order:     0
```

## Half-duplex I2S

L'implémentation `i2s_audio` d'ESPHome ne supporte pas le full-duplex sur un
même bus. Sur les plateformes avec un seul jeu de pins vers le codec (Atom
Echo S3R, etc.), le `vban_receiver` arrête le mic pendant la lecture puis le
relance au bout de `idle_timeout_ms` sans paquet.

## Devices testés

- M5Stack Atom Echo S3R (ESP32-S3, codec ES8311)

## Compatibilité receivers

- VoiceMeeter (Incoming Streams) — Banana et Potato
- VBAN Receptor (Windows / Android payant)
- VBAN Receptor X
- Wyoming-STT-VBAN addon Home Assistant
