# 🤝 Team Workflow Guide

Welcome! To ensure code stability and keep our repository clean, we follow a simple, strict Git branch and workflow system. Please read this guide before making your first commit.

---

## 🌿 The Golden Rule: Branch Protection

*   **Target Branch:** Everyone works in the **`Drone3D`** branch.
*   **⚠️ Danger Zone:** **Never push directly to `main`.** The `main` branch is reserved for final, production-ready, stabilized research milestones.

---

## 🔄 Standard Git Workflow (5 Steps)

Whenever you are working on a feature, bug fix, or documentation update, follow these five steps in your terminal:

### Step 1: Pull the Latest Changes
Before starting any work, sync your local files with the remote server to prevent merge conflicts.
```bash
git checkout Drone3D
git pull origin Drone3D
```

### Step 2: Make Your Changes
Write your code, update documentation, or edit the environment configuration.

### Step 3: Stage Your Changes
Select the files you want to save.
```bash
# Add all changes
git add .

# OR add specific files:
# git add drone_env.py
```

### Step 4: Commit Your Changes
Save your staged files with a clear, descriptive message explaining what you did.
```bash
git commit -m "feat: add weather wind placeholders and logger"
```

### Step 5: Push to the Drone3D Branch
Send your changes back to the remote repository.
```bash
git push origin Drone3D
```

---

## 🔀 The Merge Process (Simplified)

Once a major phase or research milestone is fully complete and verified on the `Drone3D` branch:

1.  **Testing Verification:** Run the simulation and Docker checks (`docker-compose up simulation` and `python setup_env.py`) to confirm everything works without errors.
2.  **Pull Request (PR):** A pull request is created on GitHub/GitLab to merge `Drone3D` into `main`.
3.  **Code Review:** At least one other team member reviews the code to ensure compatibility and that it follows the update rules.
4.  **Merge:** Once approved, the changes are merged into `main`, creating a new stable version release.

> [!TIP]
> If you encounter git conflicts, do not panic! Run `git status` to see the conflicting files, resolve the conflict markings `<<<<<<<` and `>>>>>>>` in your editor, and then proceed with `git add` and `git commit`.
