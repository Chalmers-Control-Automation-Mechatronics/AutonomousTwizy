Since this repository isn't a fork of the apollo repository, updating this repo with apollo's latest changes isn't a trivial task.

The *original* branch contains unchanged Apollo code. To update Apollo:
1. Checkout the *original* branch. `git checkout original`
2. Download a ZIP file of the Apollo repository at the release you're interested in.
3. Delete all the files in `src/BSc2018/apollo-2.0.0`
4. Extract the ZIP file into there.
5. Commit the changes. `git commit -m "Updated Apollo to something"`
6. Checkout the *master* branch. `git checkout master`
7. Merge the original branch into the master branch to apply the apollo changes to our changes. `git merge original`
8. Resolve potential merge conflicts. There's many guides on this on the internet.
9. Push the changes. `git push`
