# PowerShell Deployment Script for Physical AI Book to GitHub Pages
# This script sets up the proper environment for deployment

Write-Host "🚀 Deploying Physical AI Book to GitHub Pages..." -ForegroundColor Green

# Check if git is configured
$gitUser = $env:GIT_USER
if (-not $gitUser) {
    $gitUserName = $(git config --get user.name)
    if (-not $gitUserName) {
        Write-Warning "Git user not configured."
        Write-Host "Please set GIT_USER environment variable or configure git:"
        Write-Host "git config --global user.name 'Your Name'"
        Write-Host "git config --global user.email 'your.email@example.com'"
        Write-Host ""

        $gitUser = Read-Host "Enter your GitHub username for deployment"
        $env:GIT_USER = $gitUser
    } else {
        $env:GIT_USER = $gitUserName
        Write-Host "Using git configured username: $gitUserName" -ForegroundColor Yellow
    }
}

if (-not $env:USE_SSH) {
    Write-Host "ℹ️  Using HTTPS for GitHub access. Set USE_SSH='true' to use SSH instead." -ForegroundColor Cyan
    $env:USE_SSH = "false"
}

# Build the site first
Write-Host "🏗️  Building the site..." -ForegroundColor Yellow
try {
    npm run build
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed"
    }
} catch {
    Write-Host "❌ Build failed. Please fix errors before deploying." -ForegroundColor Red
    exit 1
}

Write-Host "✅ Build completed successfully!" -ForegroundColor Green

# Set deployment variables
$deploymentBranch = "gh-pages"
$currentBranch = $(git rev-parse --abbrev-ref HEAD)

Write-Host "📦 Preparing deployment to branch: $deploymentBranch from branch: $currentBranch" -ForegroundColor Yellow

# Run docusaurus deploy command with proper environment
Write-Host "📡 Deploying to GitHub Pages..." -ForegroundColor Magenta
try {
    npx docusaurus deploy
    if ($LASTEXITCODE -ne 0) {
        throw "Deployment failed"
    }
} catch {
    Write-Host "❌ Deployment failed: $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}

Write-Host "🎉 Deployment completed successfully!" -ForegroundColor Green
Write-Host ""
Write-Host "Your site should be available at:" -ForegroundColor White
$repoUrl = $(git config --get remote.origin.url)
if ($repoUrl) {
    $urlParts = $repoUrl -split '/'
    $username = $urlParts[$urlParts.Length - 2]
    $reponame = $urlParts[$urlParts.Length - 1] -replace '.git$', ''
    $siteUrl = "https://$($username).github.io/$($reponame)/"
    Write-Host $siteUrl -ForegroundColor Cyan

    Write-Host ""
    Write-Host "If this is your first deployment, make sure GitHub Pages is enabled in:" -ForegroundColor White
    Write-Host "Repository Settings → Pages → Source → Deploy from a branch → $deploymentBranch" -ForegroundColor White
} else {
    Write-Host "https://[your-username].github.io/[your-repository-name]/" -ForegroundColor Cyan
}