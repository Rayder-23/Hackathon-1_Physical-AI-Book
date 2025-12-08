#!/bin/bash
# Deployment script for Physical AI Book to GitHub Pages
# This script sets up the proper environment for deployment

set -e  # Exit on any error

echo "🚀 Deploying Physical AI Book to GitHub Pages..."

# Check if git is configured
if [ -z "$GIT_USER" ] && [ -z "$(git config --get user.name)" ]; then
    echo "⚠️  Warning: Git user not configured."
    echo "   Please set GIT_USER environment variable or configure git:"
    echo "   git config --global user.name 'Your Name'"
    echo "   git config --global user.email 'your.email@example.com'"
    echo ""
    read -p "Enter your GitHub username for deployment: " github_username
    export GIT_USER="$github_username"
fi

if [ -z "$USE_SSH" ]; then
    echo "ℹ️  Using HTTPS for GitHub access. Set USE_SSH=true to use SSH instead."
    export USE_SSH=false
fi

# Build the site first
echo "🏗️  Building the site..."
npm run build

if [ $? -ne 0 ]; then
    echo "❌ Build failed. Please fix errors before deploying."
    exit 1
fi

echo "✅ Build completed successfully!"

# Set deployment variables
export DEPLOYMENT_BRANCH="gh-pages"
export CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

echo "📦 Preparing deployment to branch: $DEPLOYMENT_BRANCH from branch: $CURRENT_BRANCH"

# Run docusaurus deploy command with proper environment
echo "📡 Deploying to GitHub Pages..."
npx docusaurus deploy

echo "🎉 Deployment completed successfully!"
echo ""
echo "Your site should be available at:"
echo "https://$(git config --get remote.origin.url | sed -n 's/.*:\/\/github.com\/\(.*\)\/\(.*\)\.git/\1.github.io\/\2/p')"
echo ""
echo "If this is your first deployment, make sure GitHub Pages is enabled in:"
echo "Repository Settings → Pages → Source → Deploy from a branch → $DEPLOYMENT_BRANCH"