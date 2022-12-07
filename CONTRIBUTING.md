# Adding a robot description

Before submitting a pull request, please make sure of the following:

* The corresponding description is not already in the list.
* You believe the robot description is **awesome**. This list is a curation, not a collection.
* Descriptions should come from a legal source, and be legally distributed.
* Descriptions are sorted alphabetically within each category.
* Pick the [right category](#choosing-the-right-category).
* Remove trailing whitespaces.

## Format

Use the following columns for the row of a description: ``| ROBOT-NAME | ROBOT-MAKER | [FORMAT-1](LINK-1), [FORMAT-2](LINK-2), ... | LICENSE | MESH-CHECK | INERTIA-CHECK | COLLISION-CHECK |``

* Format: can be URDF, MJCF, Xacro, etc.
* Links: point to a public repository that is, in order of preference:
    1. Up-to-date and maintained.
    2. Contains only the description (rather than a bigger project).
    3. Is the original source (rather than a fork).
* License: report the [SPDX identifier](https://spdx.org/licenses/) of the description license.
* Check the [meshes](#meshes), [inertias](#inertias) and [collisions](#collisions).

Every link and contribution, no matter how large or small, is highly appreciated and encouraged, as it helps us maintain a common repository of useful robot descriptions.

## Choosing the Right Category

Here are some rough descriptions to help you pick the right category:

- **Arms:** serial kinematic chains designed to move an end effector through a workspace.
- **Bipeds:** have two legs terminated by feet or wheels.
- **Drones:** robots that fly in the air, *a.k.a.* unmanned aerial vehicles (UAVs).
- **End Effectors:** sub-systems at the end of a limb, for example a hand at the end of an arm.
- **Mobile Manipulators:** have a wheeled mobile base and one or two arms.
- **Humanoids:** have a torso, two legs and two arms.
- **Quadrupeds:** have four legs.

## Checks

### Meshes

You can use [`yourdfpy`](https://github.com/clemense/yourdfpy/) to display a robot description. Use the ``-c`` argument to put the robot in a different configuration and check the kinematics.

### Inertias

Check that all links contain valid ``<inertial>`` tags. Also, do a quick checksum of all masses, e.g. ``cat ./some/path/robot.urdf | grep "<mass"``, and check that the sum of all values is around the expected value for the robot.

### Collisions

Check that all links contain valid ``<collision>`` tags.

## Adding a Picture to the Gallery

Optionally, you can add a picture to the gallery for a given robot description. The gallery is organized as a Markdown table of HTML linked pictures:

* Use the following cell format: ``| <img src="LINK-TO-IMAGE" alt="ROBOT-NAME" width=100> |``
* Make sure the number of cells per line stays correct. If a line is full, create a new one after it.
    * The preview feature of GitHub's editor is useful to visualize the updated table.
* The picture should be square and display well when rescaled to 100px by 100px.
* It should only depict the robot, preferably a 3D visualization over transparent background.

## Updating your PR

Making a PR adhere to the above standards can be difficult. If maintainers notice anything that needs to be improved, they will ask you to edit your PR before merging it. There's no need to open a new PR, just edit the existing one. If you're not sure how to do that, [here is a guide](https://github.com/RichardLitt/knowledge/blob/master/github/amending-a-commit-guide.md) on the different ways you can improve your PR so that it can be merged.
