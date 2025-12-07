# Quickstart: Physical AI Book (Iteration 2)

**Date**: 2025-12-08
**Feature**: Physical AI Book (Iteration 2)

## Getting Started

This guide provides a quick overview of how to set up and run the Physical AI Book (Iteration 2) project with authentication, personalization, and Urdu translation features.

### Prerequisites

- Node.js (LTS version recommended)
- npm or yarn package manager
- Git for version control
- Basic knowledge of Docusaurus and React

### Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Install additional required packages**
   ```bash
   npm install @docusaurus/core @docusaurus/preset-classic better-auth better-sqlite3
   ```

### Development Server

1. **Start the development server**
   ```bash
   npm start
   # or
   yarn start
   ```

2. **Open your browser**
   Visit `http://localhost:3000` to see the Physical AI Book

### Key Configuration Files

- `docusaurus.config.js` - Main Docusaurus configuration
- `sidebars.js` - Navigation structure
- `src/components/` - Custom components (auth, personalization, translation)
- `docs/` - Book content organized by modules

### Core Features Setup

#### Authentication
- User signup/login with better-auth
- Protected author areas
- Profile collection for personalization

#### Personalization
- User background questionnaire
- Content adaptation based on profile
- Toggle for personalized content

#### Urdu Translation
- Pre-generated Urdu MDX files
- Language toggle functionality
- Fallback to English when Urdu unavailable

### Building for Production

```bash
npm run build
# or
yarn build
```

The build artifacts will be available in the `build/` directory and can be deployed to GitHub Pages.

### Deployment to GitHub Pages

1. **Configure deployment settings** in `docusaurus.config.js`:
   ```javascript
   {
     organizationName: 'your-github-username',
     projectName: 'your-repo-name',
     deploymentBranch: 'gh-pages'
   }
   ```

2. **Deploy to GitHub Pages**:
   ```bash
   npm run deploy
   # or
   yarn deploy
   ```

### MCP Server Integration

The project integrates with Context7 MCP servers for:
- Automated content generation
- Urdu translation validation
- Sidebar structure extraction
- MDX syntax validation

### Development Workflow

1. **Content Creation**: Add new chapters to the appropriate module directory
2. **Personalization**: Add conditional content using custom React components
3. **Translation**: Generate Urdu versions and place in corresponding directories
4. **Testing**: Run `npm run build` to ensure everything works correctly
5. **Deployment**: Push changes to trigger GitHub Pages deployment

### Common Commands

- `npm start` - Start development server
- `npm run build` - Build for production
- `npm run deploy` - Deploy to GitHub Pages
- `npm run serve` - Serve built files locally for testing
- `npm run swizzle` - Customize Docusaurus components

### Troubleshooting

- **Build errors**: Check MDX syntax and file paths
- **Authentication issues**: Verify better-auth configuration
- **Translation problems**: Ensure Urdu files follow same structure as English
- **Personalization not working**: Check localStorage and component implementation