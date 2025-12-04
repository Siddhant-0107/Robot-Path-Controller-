# GitHub Push Script for Robotics Project
# Run this script in PowerShell from the project directory

# Configuration
$GITHUB_USERNAME = "Siddhant-0107"
$REPO_NAME = "Robot-Path-Controller"

# Initialize git if not already done
if (-not (Test-Path ".git")) {
    Write-Host "Initializing Git repository..." -ForegroundColor Cyan
    git init
}

# Create .gitignore if it doesn't exist
if (-not (Test-Path ".gitignore")) {
    Write-Host "Creating .gitignore..." -ForegroundColor Cyan
    @"
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
*.egg-info/
dist/
build/
eggs/
*.egg
.eggs/

# Virtual environments
.venv/
venv/
ENV/

# IDE
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db

# Output files
out/
*.csv
*.log

# Zip files
*.zip

# ROS2 build artifacts
install/
log/
build/
"@ | Out-File -FilePath ".gitignore" -Encoding UTF8
}

# Add all files
Write-Host "Adding all files..." -ForegroundColor Cyan
git add .

# Commit
Write-Host "Committing changes..." -ForegroundColor Cyan
git commit -m "Initial commit: Path smoothing and trajectory control for differential-drive robots"

# Set main branch
git branch -M main

# Add remote origin (update URL with your username)
Write-Host "Adding remote origin..." -ForegroundColor Cyan
$remoteUrl = "https://github.com/$GITHUB_USERNAME/$REPO_NAME.git"
git remote remove origin 2>$null
git remote add origin $remoteUrl

# Push to GitHub
Write-Host "Pushing to GitHub..." -ForegroundColor Yellow
Write-Host "You may be prompted for your GitHub credentials." -ForegroundColor Yellow
git push -u origin main

Write-Host "`nDone! Your code should now be on GitHub at:" -ForegroundColor Green
Write-Host "https://github.com/$GITHUB_USERNAME/$REPO_NAME" -ForegroundColor Green
