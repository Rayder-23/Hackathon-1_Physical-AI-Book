// Client-side script to handle authentication state in navbar
// This script will run after the page loads to update navbar items based on auth state

function updateAuthNavbar() {
  // Check if we're in browser environment
  if (typeof window === 'undefined' || typeof localStorage === 'undefined') {
    return;
  }

  // Get auth state from localStorage
  const authToken = localStorage.getItem('authToken');
  const currentUser = localStorage.getItem('currentUser');

  const isAuthenticated = !!(authToken && currentUser);

  // Find the dropdown menu in the navbar
  const dropdownTrigger = document.querySelector('.navbar__item.dropdown__container button.dropdown__trigger');
  if (!dropdownTrigger) return;

  // Update the dropdown label based on auth state
  if (isAuthenticated) {
    try {
      const user = JSON.parse(currentUser);
      dropdownTrigger.textContent = user.email || 'Account';
    } catch (e) {
      dropdownTrigger.textContent = 'Account';
    }
  } else {
    dropdownTrigger.textContent = 'Account';
  }

  // Find all dropdown menu items
  const dropdownMenu = document.querySelector('.navbar__item.dropdown__container .dropdown__menu');
  if (!dropdownMenu) return;

  // Get all dropdown items by their class names
  const loginItem = dropdownMenu.querySelector('a[href="/login"]');
  const registerItem = dropdownMenu.querySelector('a[href="/register"]');
  const profileItem = dropdownMenu.querySelector('a[href="/profile"]');
  const logoutItem = dropdownMenu.querySelector('a.navbar__logout-link');

  // Hide/show login/register based on auth state
  if (loginItem) {
    loginItem.style.display = isAuthenticated ? 'none' : 'block';
  }
  if (registerItem) {
    registerItem.style.display = isAuthenticated ? 'none' : 'block';
  }

  // Hide/show profile/logout based on auth state
  if (profileItem) {
    profileItem.style.display = isAuthenticated ? 'block' : 'none';
  }
  if (logoutItem) {
    logoutItem.style.display = isAuthenticated ? 'block' : 'none';
    if (isAuthenticated) {
      logoutItem.onclick = function(e) {
        e.preventDefault();
        handleLogout();
      };
    }
  }
}

function handleLogout() {
  // Call backend logout if token exists
  const token = localStorage.getItem('authToken');
  if (token) {
    // Use environment variable or default for backend URL
    const backendUrl = window.ENV?.REACT_APP_BACKEND_URL || 'http://localhost:8005';
    fetch(`${backendUrl}/api/auth/logout`, {
      method: 'POST',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    }).catch(console.error); // Handle errors silently
  }

  // Clear local state
  localStorage.removeItem('currentUser');
  localStorage.removeItem('userProfile');
  localStorage.removeItem('authToken');

  // Update navbar to reflect logout
  setTimeout(() => {
    updateAuthNavbar();
    // Redirect to home page after logout
    window.location.href = '/Hackathon-1_Physical-AI-Book/';
  }, 100);
}

// Run when DOM is loaded
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', updateAuthNavbar);
} else {
  // If DOM is already loaded, run immediately
  setTimeout(updateAuthNavbar, 100); // Small delay to ensure navbar is rendered
}

// Also run when the page is shown (in case of back/forward navigation)
window.addEventListener('pageshow', updateAuthNavbar);

// Listen for storage changes (in case of logout from another tab)
window.addEventListener('storage', (e) => {
  if (e.key === 'authToken' || e.key === 'currentUser') {
    updateAuthNavbar();
  }
});