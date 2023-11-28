# Supereight release procedure

The process to follow when making a new supereight release. The `master` branch
should always point to the latest release.



## Setup

Add the public repository as a remote to your local clone of the private
repository. This only needs to be done once for each local clone.

``` sh
git remote add public git@bitbucket.org:smartroboticslab/supereight2.git
```



## Releasing a new version

1. Run the test suite from
   [supereight-testbot](https://bitbucket.org/smartroboticslab/supereight-testbot)
   on the `devel` branch and ensure there are no issues. This typically means
   running

   ``` sh
   ./run_tests.sh -u 18.04,22.04 devel; ./run_tests.sh -d ~/Documents/Datasets/ -u 20.04 devel
   ```

1. Pull the latest commits in the `devel` and `master` branches from the private
   repository remote into your local clone.

    ``` sh
    git checkout master
    git pull origin master
    git checkout devel
    git pull origin devel
    ```

1. Increase the version number in `CMakeLists.txt` in a separate commit. Keep
   [semantic versioning](https://semver.org/) in mind when doing this.

    ``` sh
    # Set the project version in CMakeLists.txt to X.Y.Z and then:
    se2_version='vX.Y.Z'
    git add CMakeLists.txt
    git commit -m "Increment version to $se2_version"
    git push origin devel
    ```

1. Fast-forward merge the `devel` branch into the `master` branch.

    ``` sh
    git checkout master
    git merge --ff devel
    ```

1. Create a git tag with the version number with name `supereight2 vX.Y.Z` and
   the output of `git shortlog` as the extended description.

    ``` sh
    tag_msg=$(printf 'supereight2 %s\n\n%s\n' "$se2_version" "$(git shortlog origin/master..master)")
    git tag --no-sign -a -m "$tag_msg" "$se2_version" master
    ```

1. Push the master branch to both the private and public repositories.

    ``` sh
    git push origin master:master
    git push public master:master
    ```

1. Push the tag to both the private and public repositories.

    ``` sh
    git push origin "$se2_version"
    git push public "$se2_version"
    ```
