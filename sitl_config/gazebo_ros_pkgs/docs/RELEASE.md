# Releasing binaries for Gazebo ROS packages

## Official repo

The release of the official `gazebo_ros_pkgs` from this repository and the
use of the ROS buildfarm to generate the binaries is well covered by the
[ROS documentation](http://wiki.ros.org/bloom/Tutorials/ReleaseCatkinPackage).

## Unofficial gazebo versions

### Background

Each ROS release defines one version of Gazebo supported officially through
all the ROS packages. The different combinations of ROS <-> Gazebo can be
found in the [REP-3](http://www.ros.org/reps/rep-0003.html). Some examples:

 * ROS Kinetic: Gazebo 7
 * ROS Lunar: Gazebo 7
 * ROS Melodic: Gazebo 9

Some use cases require the use of alternative combinations of ROS and Gazebo
versions. The `gazebo_ros_pkgs` code is usually prepared to be compatible with
different versions of Gazebo, especially the latest ones.

To release a modified version of `gazebo_ros_pkgs` which supports a different
major version of gazebo, before running bloom some actions need to be taken:

 1. Make the system resolve the Ubuntu package `gazebo` as `gazeboX`.
 2. Fork and modify the official [gbp -release repo](https://github.com/ros-gbp/gazebo_ros_pkgs-release).
 3. Run bloom on the modified release repository.
 4. Custom infrastructure to create .deb packages.

We'll go over these steps in more detail below.

### Upstream versions released using this tutorial

The `gbp -release repository` hosts the latest version released by the maintainers
of `gazebo_ros_pkgs`. When using these instructions to release a new custom version
the version of `gazebo_ros_pkgs` released will be the latest one existing in the
official `gbp -release repository`. The version would be the same but the release number
will be one above since bloom needs to be run on top of the modified `-release repository`.

### Make the system resolve gazeboX

To have `gazebo` packages be resolved to `gazeboX` in the local
machine where bloom-release will be run, `rosdep` should be configured
accordingly.

The repository `https://github.com/osrf/osrf-rosdep/` hosts different sets
of custom rosdep keys that can be deployed in a system in order to override
the default configuration of resolving gazebo packages.

At `/etc/ros/rosdep/sources.list.d` there should be a default file named
`20-default.list`. To change the default keys by a custom version of gazebo
packages, the desired `00-gazeboX.list` from the previously indicated repository
needs to be download to the same directory.

Update the rosdep cache:

    $ rosdep update

To double check: run rosdep resolve, it should resolve to right major versions:

    $ rosdep resolve gazebo
    $ rosdep resolve gazebo_ros

### Fork and modify the gbp -release repo

The next step should be to change release metadata in order to alter `gazebo_ros_pkgs` names to include
the major version. To achieve this goal bloom provides templates (files ending with `.em`) hosted in
the own `-release repo` in branches under the following scheme `debian/<rosdistro>/<package_name>`. The
templates use [`empy`](http://www.alcyone.com/software/empy/) and will be expanded
when running bloom to generate the ubuntu/debian metadata.

First step, fork the [official release repository of gazebo_ros_pkgs](https://github.com/ros-gbp/gazebo_ros_pkgs-release).

Second step, download the renaming script, place it in the fork directory recently created and run it:

    $ cd <fork-release-directory>
    $ wget https://bitbucket.org/osrf/release-tools/raw/default/bloom/rename-gazebo-ros-pkgs.bash
    $ ./rename-gazebo-ros-pkgs.bash <unofficial_major_version> <space separated list of ROS distros>

    # example:
    #   ./rename-gazebo-ros-pkgs.bash 10 kinetic melodic


### Run Bloom on the custom -release repository

The point of this last step is to generate the ubuntu packaging metadata from bloom templates since they were
modified. The metadata will be uploaded to branches in the custom fork created before.

`bloom-release` needs to be executed once per each desired `ROS_DISTRO` to generate metadata from. The name
`gazeboX_ros_pkgs` is just a stub to avoid use the official name:

    bloom-release --track ${ROS_DISTRO} --ros-distro ${ROS_DISTRO} gazeboX_ros_pkgs --edit

Bloom will stop at some points asking for the user input:

  * `Release repository url [press enter to abort]:` the URL of the forked github repo needs to be set here
  * `Repository Name:` `gazeboX_ros_pkgs` (being `X` the custom gazebo major version)
  * The other options can be set to default just by pressing `enter`

Bloom must be stopped after generating and pushing the new tags and branches.

#### Example Releasing Gazebo8 for Kinetic

    $ bloom-release --track kinetic --ros-distro kinetic gazeboX_ros_pkgs --edit

    Specified repository 'gazeboX_ros_pkgs' is not in the distribution file located at 'https://raw.github.com/ros/rosdistro/master/kinetic/distribution.yaml'
    Did you mean one of these: 'gazebo_ros_pkgs'?
    Could not determine release repository url for repository 'gazeboX_ros_pkgs' of distro 'kinetic'
    You can continue the release process by manually specifying the location of the RELEASE repository.
    To be clear this is the url of the RELEASE repository not the upstream repository.
    For release repositories on github, you should provide the `https://` url which should end in `.git`.
    Release repository url [press enter to abort]: git@github.com:osrf/gazebo8_ros_pkgs-release.git
    The release repository url you provided is not a `https://` address.
    Would you like to enter the address again?
    Continue [Y/n]? n
    Very well, the address 'git@github.com:osrf/gazebo8_ros_pkgs-release.git' will be used as is.
    ==> Fetching 'gazeboX_ros_pkgs' repository from 'git@github.com:osrf/gazebo8_ros_pkgs-release.git'
    Cloning into '/tmp/tmpRGYh76'...
    remote: Counting objects: 16388, done.
    remote: Total 16388 (delta 0), reused 0 (delta 0)
    Receiving objects: 100% (16388/16388), 4.02 MiB | 954.00 KiB/s, done.
    Resolving deltas: 100% (4232/4232), done.
    Checking connectivity... done.
    Track 'kinetic' exists, editing...
    Repository Name:
      upstream
	Default value, leave this as upstream if you are unsure
      <name>
	Name of the repository (used in the archive name)
      ['gazebo4_ros_pkgs']:
    Upstream Repository URI:
      <uri>
	Any valid URI. This variable can be templated, for example an svn url
	can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
	where the :{version} token will be replaced with the version for this release.
      ['https://github.com/ros-simulation/gazebo_ros_pkgs.git']:
    Upstream VCS Type:
      svn
	Upstream URI is a svn repository
      git
	Upstream URI is a git repository
      hg
	Upstream URI is a hg repository
      tar
	Upstream URI is a tarball
      ['git']:
    Version:
      :{ask}
	This means that the user will be prompted for the version each release.
	This also means that the upstream devel will be ignored.
      :{auto}
	This means the version will be guessed from the devel branch.
	This means that the devel branch must be set, the devel branch must exist,
	and there must be a valid package.xml in the upstream devel branch.
      <version>
	This will be the version used.
	It must be updated for each new upstream version.
      [':{auto}']:
    Release Tag:
      :{none}
	For svn and tar only you can set the release tag to :{none}, so that
	it is ignored.  For svn this means no revision number is used.
      :{ask}
	This means the user will be prompted for the release tag on each release.
      :{version}
	This means that the release tag will match the :{version} tag.
	This can be further templated, for example: "foo-:{version}" or "v:{version}"

	This can describe any vcs reference. For git that means {tag, branch, hash},
	for hg that means {tag, branch, hash}, for svn that means a revision number.
	For tar this value doubles as the sub directory (if the repository is
	in foo/ of the tar ball, putting foo here will cause the contents of
	foo/ to be imported to upstream instead of foo itself).
      [':{version}']:
    Upstream Devel Branch:
      <vcs reference>
	Branch in upstream repository on which to search for the version.
	This is used only when version is set to ':{auto}'.
      ['kinetic-devel']:
    ROS Distro:
      <ROS distro>
	This can be any valid ROS distro, e.g. groovy, hydro
      ['kinetic']:
    Patches Directory:
      :{none}
	Use this if you want to disable overlaying of files.
      <path in bloom branch>
	This can be any valid relative path in the bloom branch. The contents
	of this folder will be overlaid onto the upstream branch after each
	import-upstream.  Additionally, any package.xml files found in the
	overlay will have the :{version} string replaced with the current
	version being released.
      [None]:
    Release Repository Push URL:
      :{none}
	This indicates that the default release url should be used.
      <url>
	(optional) Used when pushing to remote release repositories. This is only
	needed when the release uri which is in the rosdistro file is not writable.
	This is useful, for example, when a releaser would like to use a ssh url
	to push rather than a https:// url.
      [None]:
    Saving 'kinetic' track.

    ... (all bloom output while generating) ...

    <== Pushed tags successfully
    ==> Generating pull request to distro file located at 'https://raw.github.com/ros/rosdistro/master/kinetic/distribution.yaml'
    Would you like to add documentation information for this repository? [Y/n]?

    <control+C>

### Custom infrastructure to create .deb packages

All the previous steps are designed to generate the appropriate Ubuntu metadata
inside the `forked -prerelease repository`. The metadata is being hosted in
git tags following the schema:


    release/<ros_distro>/<gazebo_ros_pkg_name>/<version>
    debian/ros-<ros_distro>-<gazebo_ros_pkg_name>_<version>_<ubuntu_distro>

The `debian/...` tag contains the upstream code together with the `debian/` metadata repository ready to be
build using `debbuild` or any other debian generation tool.

For reference, the OSRF buildfarm is using [this script](https://bitbucket.org/osrf/release-tools/src/default/jenkins-scripts/docker/lib/debbuild-bloom-base.bash).
