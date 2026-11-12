# 2702 Rebels: public projects

Code, CAD, and other work from [FRC Team 2702 (Rebels)](https://2702rebels.com) that we share with the FIRST community.

Each folder is a copy of one of the team's own repositories, updated automatically once a day. Only the current files are copied, not the history.

Everything here is under the [BSD 3-Clause license](LICENSE) unless a folder has its own license file, which covers that folder. Third-party files keep their original licenses.

Files too big for git, like robot CAD exports, are on the [Releases](../../releases) page.

## Projects

<!-- projects:start -->
| Project | What it is | Updated |
| --- | --- | --- |
| [2022-robot](2022-robot/) | Robot code for the 2022 season (RAPID REACT) | 2023-11-16 (`834c84c`) |
| [2026-robot](2026-robot/) | Robot code for the 2026 season (REBUILT) | 2026-10-25 (`b4979c7`) |
<!-- projects:end -->

Questions or ideas? Open an issue.

---

## For team members

### Publishing a project

1. In the source repo, check that nothing private is in it: passwords, API keys, team or student personal info, sponsor details. History doesn't come along, but every current file does.
2. To keep a file or folder private while publishing the rest, either mark it in the source repo's `.gitattributes`:

   ```
   notes/ export-ignore
   ```

   or list it under `exclude` in [`sources.yml`](sources.yml).
3. Open a PR here that adds the project to [`sources.yml`](sources.yml):

   ```yaml
   projects:
     - repo: 2026-robot
       ref: master
       releases: false
       description: Robot code for the 2026 season (REBUILT)
   ```

   Always set `ref` and `releases`, even when the default would do: say which branch to publish (often not the repo's default) and whether its releases come along. The field list is at the top of that file. The project is published under this repo's BSD 3-Clause license; to use a different one, add a `LICENSE` file to the source repo and it comes along with the folder. Once the PR merges, the Sync workflow copies the project in a few minutes.

To unpublish, delete the entry. The folder is removed on the next sync, but anything already published stays in this repo's history and may already have been copied elsewhere.

Every sync scans for secrets with [gitleaks](https://github.com/gitleaks/gitleaks), and a project with findings isn't updated until they're fixed. That check is a safety net: step 1 is still your job.

### Publishing CAD and other large files

- **From a project's own releases:** set `releases: true` on its entry. Each release in the source repo, with its attached files, is copied here, tagged `<folder>/<tag>` (for example `2026-robot/v1.0`). New releases are picked up by the next sync after they're published, whenever that is. Once a release has been copied it isn't updated, so attach all its files before you publish it (drafts are skipped), and put any file you missed in a new release.
- **By hand:** create a release on this repo and attach the files (up to 2 GB each). Name the tag `<project>/<what>`, like `2026-robot/cad-champs`, so it sorts with its project.

Git LFS files aren't copied, so a project that uses LFS needs those files excluded and published as release files.

### How it works

[`.github/workflows/sync.yml`](.github/workflows/sync.yml) runs [`scripts/sync.py`](scripts/sync.py) once a day, whenever `sources.yml` changes on main, and on demand from the Actions tab (Run workflow, optionally for one folder). For each project it fetches the source's branch tip, copies the files, scans them, and commits the folder if anything changed. [`.github/sync-state.json`](.github/sync-state.json) records which source commit each folder came from.

It reads the private repos through the **2702rebels-public-sync** GitHub App, which has read-only access to repository contents. Each run asks for a token limited to the repos in `sources.yml`. It pushes with a deploy key that's allowed past the review rule on main. Everyone else changes main through a reviewed PR, which is what keeps `sources.yml` from publishing a repo without a second person agreeing.

To try a sync locally without committing:

```bash
pip install pyyaml
SOURCE_TOKEN=$(gh auth token) python3 scripts/sync.py --no-commit --only 2026-robot
```
