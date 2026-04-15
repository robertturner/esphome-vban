# esphome-vban

Composant ESPHome custom pour envoyer le flux audio d'un microphone I2S via VBAN (UDP).
Supporte le multicast pour diffuser vers plusieurs receivers simultanément (ClapTrap, etc.).

## Installation

```yaml
external_components:
  - source: github://powange/esphome-vban
    components: [vban_sender]
```

## Usage

```yaml
vban_sender:
  target_ip: "239.0.0.1"   # multicast
  target_port: 6980
  stream_name: "AtomBureau"
```

Puis dans une `interval` pour lire le micro et envoyer en VBAN :

```yaml
interval:
  - interval: 20ms
    then:
      - lambda: |-
          std::vector<int16_t> samples;
          id(atom_mic).read(samples, 320);
          if (!samples.empty()) {
            id(vban).send_audio(
              (const uint8_t*)samples.data(),
              samples.size() * 2
            );
          }
```

## Paramètres

| Paramètre | Requis | Défaut | Description |
|---|---|---|---|
| `target_ip` | ✅ | - | IP destination (unicast ou multicast `239.0.0.1`) |
| `target_port` | ❌ | `6980` | Port UDP VBAN |
| `stream_name` | ❌ | `AtomEcho` | Nom du stream VBAN (16 chars max) |

## Format audio

- Sample rate : 16000 Hz
- Format : PCM 16-bit mono
- Compatible ClapTrap, Voicemeeter, VBAN Receptor

## Devices testés

- M5Stack Atom Echo S3R
