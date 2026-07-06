# MEMOIRE.md — mémoire persistante claude-mem (installée le 06-07)

Système de mémoire inter-sessions pour Claude Code : plugin
[claude-mem](https://github.com/thedotmack/claude-mem) v13.10.2. Les sessions sont observées
par des hooks (SessionStart, PreToolUse, PostToolUse, Stop…), compressées en « observations »
SQLite, et réinjectées automatiquement au démarrage des sessions suivantes.

## Points clés pour CE projet

- **Multi-branche natif** : la mémoire est indexée par RÉPERTOIRE projet (`sonar-SLAM/`),
  pas par branche git → Bruce, Bruce_Sonar_USBL, caves et holoocean partagent le même
  historique de mémoire. Exactement ce qu'on veut pour un stage multi-branches.
- **Machine-locale, jamais versionnée** : tout vit dans `~/.claude-mem/` (SQLite). Rien dans
  le dépôt, rien à synchroniser entre branches. Ce fichier-ci n'est que le mode d'emploi.
- **L'injection commence à la 2ᵉ session** dans le projet (la 1ʳᵉ ne fait qu'enregistrer).
- La mémoire **native** de Claude Code (`memory/` + MEMORY.md côté agent) reste active —
  les deux se complètent (choix fait à l'installation).

## Affichage graphique (web viewer)

- **http://127.0.0.1:37700** — flux temps réel des observations pendant que Claude travaille.
- ⚠ Le README GitHub annonce le port 37777 : obsolète, la v13 sert sur **37700**.
- Le worker est (re)démarré automatiquement par le hook SessionStart à chaque session
  Claude Code. À la main si besoin : `claude-mem start` / `status` / `stop`.

## Usage dans Claude Code

- **Recherche** : skill `mem-search` — interroger l'historique en langage naturel, en
  3 couches économes en tokens : `search` (index compact) → `timeline` (contexte
  chronologique) → `get_observations` (détails complets). Exemple : « cherche dans la
  mémoire ce qu'on a conclu sur le seuil CFAR caves ».
- **Amorçage optionnel** : `/learn-codebase` ingère tout le repo (~5 min) au lieu
  d'attendre que la mémoire se construise passivement.
- Autres skills installés (mêmes auteurs) : `timeline-report`, `standup`, `what-the`, etc.

## Installation faite (pour dépannage)

- Node 24.18.0 LTS **userspace** : `~/.local/opt/node` (symlinks node/npm/npx/claude dans
  `~/.local/bin`) — pas de sudo, indépendant de dnf.
- CLI `claude` installé via npm (requis par le worker pour compresser) ; chemin figé dans
  `~/.claude-mem/settings.json` → `CLAUDE_CODE_PATH`.
- Bun 1.3.14 + uv (recherche vectorielle) auto-installés par l'installateur.
- Plugin : `~/.claude/plugins/cache/thedotmack/claude-mem/13.10.2` (marketplace thedotmack).
- Piège connu : `claude-mem restart` ne tue pas toujours l'ancien worker (« port still
  bound ») → `kill $(python3 -c "import json;print(json.load(open('$HOME/.claude-mem/worker.pid'))['pid'])")`
  puis `claude-mem start`.
- Santé : `curl -s http://127.0.0.1:37700/api/health` → doit dire `"degraded": false`.
