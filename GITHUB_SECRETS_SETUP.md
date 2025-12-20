# GitHub Actions Secrets Configuration

This document explains how to set up the GitHub Actions secret for `REACT_APP_BACKEND_URL` to connect your GitHub Pages frontend to your Railway backend.

## Setting Up the GitHub Secret

### Step 1: Access Repository Settings
1. Go to your GitHub repository
2. Click on "Settings" tab
3. In the left sidebar, click on "Secrets and variables"
4. Click on "Actions"
5. Click "New repository secret"

### Step 2: Add the Secret
- **Name**: `REACT_APP_BACKEND_URL`
- **Value**: `https://[REDACTED].railway.app` (your Railway domain)

### Step 3: Save the Secret
Click "Add secret" to save.

## How It Works

The updated GitHub Actions workflow (in `.github/workflows/deploy.yml`) now:

1. **During the build process**, reads the secret value
2. **Creates a .env.production file** with the backend URL
3. **Builds the React app** with the environment variable embedded
4. **Deploys the built files** to GitHub Pages

## Workflow Configuration

The workflow file now includes:

```yaml
- name: Build
  run: |
    echo "REACT_APP_BACKEND_URL=${{ secrets.REACT_APP_BACKEND_URL }}" > .env.production
    npm run build
  env:
    REACT_APP_BACKEND_URL: ${{ secrets.REACT_APP_BACKEND_URL }}
```

This ensures that the `REACT_APP_BACKEND_URL` is available at build time and gets embedded in your React application.

## Important Notes

1. **Build Time vs Runtime**: Environment variables are embedded at build time, not runtime
2. **Secret Security**: The secret value is only available during the GitHub Actions workflow run
3. **Public Information**: Since your Railway domain is public, you could also hardcode it in the workflow if preferred
4. **Flexibility**: Using a secret allows you to change the backend URL without modifying the workflow file

## Alternative: Hardcoded Value

If you prefer to hardcode the value directly (since it's public information), you can modify the workflow to:

```yaml
- name: Build
  run: |
    echo "REACT_APP_BACKEND_URL=https://[REDACTED].railway.app" > .env.production
    npm run build
```

## Verification

After setting up the secret and pushing changes to your repository:

1. The GitHub Actions workflow will run on the next push to main
2. Check the workflow logs to ensure the build completes successfully
3. Visit your GitHub Pages site to verify it connects to your Railway backend
4. Test the chat functionality to ensure API calls are working

## Troubleshooting

If the frontend doesn't connect to your backend:

1. Check that your Railway backend is deployed and running
2. Verify the backend URL in your GitHub secret is correct
3. Check browser console for CORS or network errors
4. Ensure the backend allows requests from your GitHub Pages domain