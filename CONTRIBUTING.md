# Contributions Guidelines

Before submitting a pull request, please make sure of the following:

* The corresponding description is not already in the list.
* You believe the robot description is **awesome**. This list is a curation, not a collection.
* Use the following columns for the row of a description: ``| ROBOT-NAME | ROBOT-MAKER | [FORMAT-1](LINK-1), [FORMAT-2](LINK-2), ... | LICENSE | NB-DOFS | MASS |``
* Links should point to a public repository that is, in order of preference:
    1. Up-to-date and maintained.
    2. Contains only the description (rather than a bigger project).
    3. Is the original source (rather than a fork).
* Format can be URDF, MJCF, XML, ...
* Use only two significant digits for the mass.
* Descriptions should come from a legal source, and be legally distributed.
* Descriptions are sorted alphabetically within each category.
* Remove trailing whitespaces.

Every link and contribution, no matter how large or small, is highly appreciated and encouraged, as it helps us maintain a common repository of useful robot descriptions.

## Adding a Picture to the Gallery

Optionally, you can add a picture to the gallery for a given robot description. The gallery is organized as a Markdown table of HTML linked pictures:

* Use the following cell format: ``| <a href="LINK-TO-DESCRIPTION"><img src="LINK-TO-IMAGE" alt="ROBOT-NAME" width=100></a> |``
* Make sure the number of cells per line stays correct. If a line is full, create a new one after it.
    * The preview feature of GitHub's editor is useful to visualize the updated table.
* The picture should be square and display well when rescaled at 100px by 100px.
* It should only depict the robot, preferably a 3D visualization over transparent background.

## Updating your PR

Making a PR adhere to the above standards can be difficult. If maintainers notice anything that needs to be improved, they will ask you to edit your PR before merging it. There's no need to open a new PR, just edit the existing one. If you're not sure how to do that, [here is a guide](https://github.com/RichardLitt/knowledge/blob/master/github/amending-a-commit-guide.md) on the different ways you can improve your PR so that it can be merged.
