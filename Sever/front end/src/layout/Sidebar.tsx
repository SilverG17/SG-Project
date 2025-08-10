// src/layout/Sidebar.tsx
import React from 'react';
import { Link } from 'react-router-dom';

const Sidebar = () => (
  <div className="w-64 h-screen bg-gray-800 text-white p-4">
    <h2 className="text-xl font-bold mb-6">Dashboard</h2>
    <nav>
      <ul>
        <li><Link to="/" className="block py-2">Home</Link></li>
        <li><Link to="/analytics" className="block py-2">Analytics</Link></li>
        <li><Link to="/settings" className="block py-2">Settings</Link></li>
      </ul>
    </nav>
  </div>
);

export default Sidebar;