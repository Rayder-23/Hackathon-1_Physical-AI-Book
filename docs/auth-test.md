---
title: Authentication Test Page
---

# Authentication Test Page

This page demonstrates the authentication features of the Physical AI Book.

## Auth Status

import { useAuth } from '@site/src/contexts/AuthContext';

<div id="auth-status">
  {typeof window !== 'undefined' && (
    <script
      dangerouslySetInnerHTML={{
        __html: `
          const user = JSON.parse(localStorage.getItem('currentUser') || 'null');
          const isAuthenticated = !!user;
          document.getElementById('auth-status').innerHTML =
            '<p><strong>Authenticated:</strong> ' + isAuthenticated + '</p>' +
            (user ? '<p><strong>User:</strong> ' + user.email + ' (Role: ' + (user.role || 'user') + ')</p>' : '');
        `
      }}
    />
  )}
</div>

## Login

import { Login } from '@site/src/components/Auth/Login';

<Login onLoginSuccess={(user) => console.log('Login successful:', user)} />

## Register

import { Register } from '@site/src/components/Auth/Register';

<Register onRegisterSuccess={(user) => console.log('Registration successful:', user)} />

## Protected Content Example

import { ProtectedRoute } from '@site/src/components/Auth/ProtectedRoute';

<ProtectedRoute>
  <div className="protected-content">
    <h3>Protected Content</h3>
    <p>This content is only visible to authenticated users.</p>
    <p>Your authentication is working correctly!</p>
  </div>
</ProtectedRoute>

## Author-Only Content Link

[Access Author Content](./author/introduction.md) - This link should only work for authenticated authors.