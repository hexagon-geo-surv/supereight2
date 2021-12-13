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
   on the `devel` branch and ensure there are no issues.
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
1. Fast-forward merge the `devel` branch into the `master` branch.

    ``` sh
    git checkout master
    git merge --ff devel
    ```
1. Create a git tag with the version number.

    ``` sh
    git tag vX.Y.Z master
    ```
1. Push the master branch to both the private and public repositories.

    ``` sh
    git push origin master:master
    git push public master:master
    ```
1. Push the tag to both the private and public repositories.

    ``` sh
    git push origin vX.Y.Z
    git push public vX.Y.Z
    ```

