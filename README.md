## Autonomous Twizy

### To contribute
Create an organization for your group https://help.github.com/articles/creating-a-new-organization-from-scratch/ and then in https://github.com/Chalmers-Control-Automation-Mechatronics/AutonomousTwizy click the **Fork** button top-right on your screen. Fork into the organization you created.

You will now have your own version of this repository and you can invite your group members to your organization in order to collaborate with the group. Since you have your own repository, you won't see changes other groups make automatically. This is intended in order to reduce code conflicts between the group.

#### Updating the main repository with your group's changes
When you have done changes that are working and you want to share them with the rest of the Twizy team, you can create a **Pull Request**. To do that, go into **your** organizations repository, and click the **Pull Requests** tab, then click **New pull request**. Make sure _base fork_ is set as `Chalmers-Control-Automation-Mechatronics/AutonomousTwizy` and _base_ is `master`. Also make sure _head fork_ is set as your repository. Here you can now review your changes and create a nice title and description of what you have done. A maintainer can now review this and accept the pull request, which will merge your changes into the main repository.

#### Updating your group's repository with the latest code in the main repository
https://help.github.com/articles/merging-an-upstream-repository-into-your-fork/

### Git structure
* The *master* branch in https://github.com/Chalmers-Control-Automation-Mechatronics/AutonomousTwizy is intended to be where all the changes to Apollo for the Twizy project takes place.
* The *original* branch contains unchanged Apollo code. Read the `Update Apollo.md` guide on how to update Apollo.
