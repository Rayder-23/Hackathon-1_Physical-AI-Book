---
title: Authentication Test Page
---

# Authentication Test Page

This page demonstrates the authentication features of the Physical AI Book.

## Auth Status

<div id="auth-status">
  <p><strong>Authentication Status:</strong> Check console or login to see status</p>
</div>

## Login

import { Login } from '../src/components/Auth/Login';

<Login onLoginSuccess={(user) => console.log('Login successful:', user)} />

## Register

import { Register } from '../src/components/Auth/Register';

<Register onRegisterSuccess={(user) => console.log('Registration successful:', user)} />

## Protected Content Example

import { ProtectedRoute } from '../src/components/Auth/ProtectedRoute';

<ProtectedRoute>
  <div className="protected-content">
    <h3>Protected Content</h3>
    <p>This content is only visible to authenticated users.</p>
    <p>Your authentication is working correctly!</p>
  </div>
</ProtectedRoute>

## Author-Only Content Link

[Access Author Content](./author/introduction.md) - This link should only work for authenticated authors.