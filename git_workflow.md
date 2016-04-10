# Ubiquity Robotics `git` Workflow

`git` provides a powerful and flexible version management system.
Rather than supporting a rigid workflow, `git` supports a multitude
of different workflows.  The question arises of which of the many
workflow options should Ubiquity Robotics use?  This document
proposes a reasonable workflow.

## `git` Overview

`git` is basically a transactional file system.  The `git` transactional
file system is stored on a `git` repository (e.g. `github.com` or
`bitbucket.com`.)  "Transactional" is a database concept whereby all
changes appear to occur atomically.  Either all the changes occur or
none of them do.  Each time you make some changes to a git repository,
it creates a "commit" which is a snap-shot of all of the files at the
time of the commit.  Each commit has a "unique" 160-bit SHA1 hash
computed on its contents.  The likelihood of two commits having the
same hash is remote.  Commits can also have release name and branch
names attached to them.  While the release names tend to be quite
sticky, the branch names frequently move over time.  Each commit points
to its predecessor commit.  In the case of a merge, the commit points
to both of its predecessors.

In general, each person that is working on a project shares a copy of
the `git` repository.  The primary benefit of this strategy is that
it allows people to work without having to have a centralized repository
control (e.g. CVS, Perforce, etc.)

`git` allows for a multitude of workflows.  This primarily done with
the concept of a branch.  Each branch can be worked on independently.
At the user's desecration, an other branch can be merged into a branch
of their choosing.

## Workflows

The simplest workflow is where you have one master branch and
each developer checks their code directly into the master.
The problem with this workflow is that it is very easy to
"break the master" by checking in broken code.  This can
cause great consternation as the broken code propogates to
other developers.

As more developers pile onto a project, the single master workflow
starts to break down.  The next workflow is a nested workflow
where there are one or more integration branches that people
merge to.  When the code in the integration branch is deemed
to be mature, it can be merged up to the top level master branch.
Some organizations use a rigid structure where each developer
can only contribute to a single integration branch.  Others
are more flexible and allow a single developer to contribute
to multiple integration branches.

## Proposed Ubiquity Robotics Workflow

Since Ubiquity Robotics is based on ROS (Robot Operating System)
from OSRF (Open Source Robotics Foundation), we need to adopt a
workflow that is compatible with the OSRF workflow.  The OSRF
workflow is that they release one master branch per year.  Each
OSRF ROS release one is given a name -- Indigo, Jade, Kinetic, etc.
The releases that occur on even numbered years are synchronized
to the Ubuntu LTS (Long Term Support) releases.  Thus, Indigo is
synchronized with Ubuntu 14.04LTS (2014.April Long Term Support.)
Kinetic will be synchronized with Ubuntu 16.04LTS.

In general, Ubiquity Robotics currently has inadequate staffing
to support the odd year number releases.  Thus, we will tend to
have two master releases at a time.  For now, we would have
`indigo_master` and fairly shortly we will probably open
`kinetic_master`.  We can have Ubiquity Robotics specific releases
are branched off one of these master releases.

We can have as many integration branches as we want.  Each integration
branch should be for a specific set of features or bug fixes.
For example, the unified launch files effort of Wayne and Rohan
belongs in its own integration branch.  Similarly, we should
have an integration area for each Ubiquity Robotics software
release.  For example, the last release was based on the image
that Wayne developed in mid-Octobler of 2015.

The real issue is deciding software can be merged from integration
branch to another.

## Testing

In general, each integration area will have a criterion that must
be met before it can be merged to.  The criterion can range from
minimal to extreme:

* No criterion: Just fling it in.

* Documentation: Some documentation is written.

* Compiles: The code compiles but may not work.

* Unit Tests: Passes unit tests.

* Smoke Test: The code passes a minimal test suite.

* Extensive Test Suite: The code passes an extensive test suite.

In addition, the criterion can change over time.  The pre-alpha, the
criterion can be at its lowest, during alpha the bar can raise a little,
and during beta even higher, etc.

What testing criterion should Ubiquity Robotics aspire to?
In general, the "no regressions" criteria is that the new
code does not break previously merged functionality.  Thus,
when we get the Raspberry Pi camera to work, in each subsequent
release it will continue to work.

## Synchronizing Multiple Repositories

The way that OSRF has structured ROS is with one `git` repository
per ROS package.  The problem with having multiple repositories,
is that it breaks atomicity.  Each `git` repository is individually
atomic, but a group of them together is not.

There are at least three technologies for dealing with multiple
repositories using `git`:

* [git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules):
  This one is actually pretty complicated.  There are
  [complaints](https://codingkilledthecat.wordpress.com/2012/04/28/why-your-company-shouldnt-use-git-submodules/)
  about sub-modules.

* [git subtrees](https://github.com/apenwarr/git-subtree/blob/master/git-subtree.txt):
  This one is simpler than sub-modules, but is still pretty complicated.
  In order to install a newer version of git that

        sudo add-apt-repository ppa:git-core/ppa
        sudo apt-get update
        sudo apt-get install git

* [git release tags](https://git-scm.com/book/en/v2/Git-Basics-Tagging):
  With this strategy, we sweep through all of our repositories and
  apply the same release tag to a specific commit in each specific
  repository.  With a shell script, this can be easily implemented.

At this point in time, it is not clear which of these techniques
we should use.  We do need to do something.  Wayne is leaning
towards release tags, but sub-trees with some simple shell scripts
might do the trick.

## Specific Proposal

The current proposal is as follows:

* Each Ubiquity Robotics repository will have both a `indigo` and
  a `kinetic` branch.  Until `kinetic` is actually being worked on, we
  can skip the `kinetic` branch.

* It is proposed that each specific release name be of the form `urX.Y.Z`,
  where `X` is the major release, `Y` is the minor release, and
  `Z` is an additional dot release.  We may choose to encode development
  level into 'Y' (e.g. 0 = open development with regressions allowed,
  2 = initial functionality with no regressions from previous release,
  4 = pre-alpha, 6 = alpha development, 8 = beta development, 10 = actual
  release, 11 = first bug fix release, 12 = second bug fix release, etc.)

* Each major Ubiquity Robotics release will have a branch name as well.
  These will be called `indigo_RELEASE` and `kinetic_RELEASE`, where
  RELEASE number will only contain the `X` portion of the release.
  Thus, we will have an `indigo_ur1` branch for the first major release,
  and `indigo_ur2` for second, etc.

* We will construct integration branches as needed.  The two main
  integration branchs will be called `indigo_devel` and `kinetic_devel`.

We need to start thinking about testing strategies.  It is recommended
that we use py.test whenever possible:

* Unit Tests:  Whenever possible we should create unit tests.

* Simulator Tests: We should create some tests that exercise as much as
  possible using a simulator.

* Documentation: I think we need to be pretty firm about disallowing
  up merging until the new features are documented.

* Smoke Test: A smoke test ensures that each major piece of functionality
  we deploy is tested.  This will almost certainly be a manual test,
  so it is important that we keep the test time down to an hour or so.

