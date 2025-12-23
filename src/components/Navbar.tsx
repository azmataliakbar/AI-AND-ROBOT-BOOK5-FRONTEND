// Navbar.tsx - COMPLETE FINAL VERSION - 270PX OPTIMIZED
import React, { useState } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import { useLocation } from '@docusaurus/router';
import styles from './Navbar.module.css';

type NavbarItem = {
  label: string;
  to: string;
  emoji?: string;
};

const NAVBAR_ITEMS: NavbarItem[] = [
  { label: 'Home', to: '/', emoji: '🏠' },
  { label: 'Book', to: '/docs/intro', emoji: '📚' },
  { label: 'Blog', to: '/blog', emoji: '📝' },
  { label: 'Chat', to: '/chat', emoji: '💬' },
];

export default function Navbar(): React.ReactElement {
  const location = useLocation();
  const [isMenuOpen, setIsMenuOpen] = useState(false);
  const [searchQuery, setSearchQuery] = useState('');

  console.log('🔍 Navbar rendering. Menu state:', isMenuOpen);

  const isActive = (item: NavbarItem): boolean => {
    return location.pathname === item.to || location.pathname.startsWith(item.to + '/');
  };

  const handleSearch = (e: React.FormEvent) => {
    e.preventDefault();
    if (searchQuery.trim()) {
      window.location.href = `/search?q=${encodeURIComponent(searchQuery)}`;
    }
  };

  return (
    <nav className={'navbar ' + styles.navbar}>
      <div className={styles.navInner}>
        {/* Left Side: Logo + Desktop Nav */}
        <div className={styles.leftSide}>
          <Link className={styles.brand} to="/">
            <span className={styles.robotIcon}>🤖</span>
            <span className={styles.brandText}>AI Robotics Book</span>
          </Link>

          {/* Desktop Nav Links */}
          <div className={styles.navLinks}>
            <Link to="/" className={clsx(styles.navLink, isActive(NAVBAR_ITEMS[0]) && styles.active)}>
              <span className={styles.navEmoji}>🏠</span>Home
            </Link>
            <Link to="/docs/intro" className={clsx(styles.navLink, isActive(NAVBAR_ITEMS[1]) && styles.active)}>
              <span className={styles.navEmoji}>📚</span>Book
            </Link>
            <Link to="/blog" className={clsx(styles.navLink, isActive(NAVBAR_ITEMS[2]) && styles.active)}>
              <span className={styles.navEmoji}>📝</span>Blog
            </Link>
            <Link to="/chat" className={clsx(styles.navLink, isActive(NAVBAR_ITEMS[3]) && styles.active)}>
              <span className={styles.navEmoji}>💬</span>Chat
            </Link>
          </div>
        </div>

        {/* Right Side: Desktop Search */}
        <div className={styles.rightSide}>
          <form onSubmit={handleSearch} className={styles.searchForm}>
            <input
              type="text"
              value={searchQuery}
              onChange={(e) => setSearchQuery(e.target.value)}
              placeholder="Search..."
              className={styles.searchInput}
            />
            <button type="submit" className={styles.searchButton}>
              🔍
            </button>
          </form>
        </div>

        {/* Mobile Toggle Button */}
        <button
            className={styles.mobileToggle}
            onClick={() => {
              console.log('🍔 Hamburger clicked! Changing from', isMenuOpen, 'to', !isMenuOpen);
              setIsMenuOpen(!isMenuOpen);
            }}
            type="button"
            style={{
              fontSize: '1.5rem',
              padding: '0.1rem 0.4rem',
              minWidth: '30px',
              minHeight: '30px'
            }}
          >
            {isMenuOpen ? '✕' : '☰'}
          </button>
      </div>

      {/* Mobile Menu - OPTIMIZED FOR 270PX */}
      <div 
        style={{ 
          display: isMenuOpen ? 'block' : 'none',
          background: 'white',
          padding: '0.75rem',
          borderTop: '3px solid #0ea5e9',
          width: '100%',
          position: 'absolute',
          left: 0,
          right: 0,
          top: '100%',
          zIndex: 9999,
          boxShadow: '0 4px 12px rgba(0,0,0,0.2)',
          boxSizing: 'border-box',
          maxHeight: 'calc(100vh - 70px)',
          overflowY: 'auto'
        }}
      >
        {/* Debug Banner */}
        <div style={{ 
          background: isMenuOpen ? '#10b981' : '#ef4444',
          color: 'white',
          padding: '6px',
          marginBottom: '8px',
          borderRadius: '4px',
          textAlign: 'center',
          fontWeight: 'bold',
          fontSize: '12px'
        }}>
          MENU IS {isMenuOpen ? 'OPEN ✅' : 'CLOSED ❌'}
        </div>

        {/* Mobile Search Form */}
        <form 
          onSubmit={(e) => {
            handleSearch(e);
            setIsMenuOpen(false);
          }} 
          style={{
            display: 'flex',
            gap: '0.4rem',
            marginBottom: '0.75rem'
          }}
        >
          <input
            type="text"
            value={searchQuery}
            onChange={(e) => setSearchQuery(e.target.value)}
            placeholder="Search..."
            style={{
              flex: 1,
              padding: '0.6rem',
              border: '2px solid #e2e8f0',
              borderRadius: '6px',
              fontSize: '0.9rem',
              outline: 'none',
              minWidth: 0
            }}
          />
          <button 
            type="submit"
            style={{
              padding: '0.6rem 1rem',
              background: '#0ea5e9',
              color: 'white',
              border: 'none',
              borderRadius: '6px',
              fontSize: '1.1rem',
              cursor: 'pointer',
              flexShrink: 0
            }}
          >
            🔍
          </button>
        </form>
        
        {/* Mobile Navigation Links */}
        <a 
          href="/"
          onClick={(e) => {
            e.preventDefault();
            setIsMenuOpen(false);
            window.location.href = '/';
          }}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '0.4rem',
            padding: '0.65rem 0.75rem',
            background: '#f0f9ff',
            marginBottom: '0.4rem',
            borderRadius: '6px',
            textDecoration: 'none',
            color: '#0f172a',
            fontSize: '14px',
            fontWeight: '600',
            border: '2px solid #e0f2fe',
            cursor: 'pointer'
          }}
        >
          <span style={{ fontSize: '1.1rem', flexShrink: 0 }}>🏠</span>
          Home
        </a>
        <a 
          href="/docs/intro"
          onClick={(e) => {
            e.preventDefault();
            setIsMenuOpen(false);
            window.location.href = '/docs/intro';
          }}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '0.4rem',
            padding: '0.65rem 0.75rem',
            background: '#f0f9ff',
            marginBottom: '0.4rem',
            borderRadius: '6px',
            textDecoration: 'none',
            color: '#0f172a',
            fontSize: '14px',
            fontWeight: '600',
            border: '2px solid #e0f2fe',
            cursor: 'pointer'
          }}
        >
          <span style={{ fontSize: '1.1rem', flexShrink: 0 }}>📚</span>
          Book
        </a>
        <a 
          href="/blog"
          onClick={(e) => {
            e.preventDefault();
            setIsMenuOpen(false);
            window.location.href = '/blog';
          }}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '0.4rem',
            padding: '0.65rem 0.75rem',
            background: '#f0f9ff',
            marginBottom: '0.4rem',
            borderRadius: '6px',
            textDecoration: 'none',
            color: '#0f172a',
            fontSize: '14px',
            fontWeight: '600',
            border: '2px solid #e0f2fe',
            cursor: 'pointer'
          }}
        >
          <span style={{ fontSize: '1.1rem', flexShrink: 0 }}>📝</span>
          Blog
        </a>
        <a 
          href="/chat"
          onClick={(e) => {
            e.preventDefault();
            setIsMenuOpen(false);
            window.location.href = '/chat';
          }}
          style={{
            display: 'flex',
            alignItems: 'center',
            gap: '0.4rem',
            padding: '0.65rem 0.75rem',
            background: '#f0f9ff',
            borderRadius: '6px',
            textDecoration: 'none',
            color: '#0f172a',
            fontSize: '14px',
            fontWeight: '600',
            border: '2px solid #e0f2fe',
            cursor: 'pointer'
          }}
        >
          <span style={{ fontSize: '1.1rem', flexShrink: 0 }}>💬</span>
          Chat
        </a>
      </div>
    </nav>
  );
}