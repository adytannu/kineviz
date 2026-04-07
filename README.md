# EchoShield INS Offsets Visuallizer

Static GitHub Pages site for visualizing EchoShield roll, pitch, and yaw
offsets relative to an INS.

## Publishing

Configure GitHub Pages to publish from:

- Branch: `main`
- Folder: `/docs`

Set the custom domain to:

- `kinematics.tannu.me`

The site domain file is committed at [docs/CNAME](/home/adysan/git/kineviz/docs/CNAME).

## Local Preview

From this directory:

```bash
python3 -m http.server 8123 --directory docs
```

Then open `http://127.0.0.1:8123/`.
