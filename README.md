# Bunnybots 2023
Our source code for our veteran-led Bunnybot for our 2023 Bunnybots Game: Hibernation Harvest!

Credit to 6328 for characterization commands and analysis.

-----------------------------

## Workflows

There are currently three workflows, `run-tests.yml` to run tests, `ktlint.yml` to check formatting, and `gen-docs.yml` to generate documentation. Here is how `gen-docs.yml` works:

- It's triggered whenever you push to `main`
- It runs `./gradlew dokkaHtml` to generate HTML from all our doc comments
- The generated HTML is uploaded and deployed to GitHub Pages
- The documentation is then accessible at https://blair-robot-project.github.io/framework2023/.
