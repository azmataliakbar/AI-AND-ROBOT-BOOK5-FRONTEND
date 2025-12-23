// SearcBar.tsx

import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './SearchBar.module.css';

type SearchSuggestion = {
  id: string;
  title: string;
  content_preview: string;
  module: number;
};

// ✅ Make sure these props are defined
type SearchBarProps = {
  onSearch?: (query: string) => void;
  placeholder?: string;
};

// ✅ Make sure function signature includes props
function SearchBar({ 
  onSearch, 
  placeholder = "Search the robotics book..." 
}: SearchBarProps): React.ReactElement {
  const [query, setQuery] = useState('');
  const [suggestions, setSuggestions] = useState<SearchSuggestion[]>([]);
  const [showSuggestions, setShowSuggestions] = useState(false);
  const [activeSuggestionIndex, setActiveSuggestionIndex] = useState(-1);
  const searchRef = useRef<HTMLDivElement>(null);

  const mockSuggestions: SearchSuggestion[] = [
    {
      id: 'ch_001',
      title: 'Introduction to ROS 2',
      content_preview: 'Robot Operating System 2 (ROS 2) is a flexible framework...',
      module: 1
    },
    {
      id: 'ch_002',
      title: 'Nodes and Topics',
      content_preview: 'In ROS 2, a node is a process that performs computation...',
      module: 1
    },
    {
      id: 'ch_013',
      title: 'Introduction to Gazebo Simulation',
      content_preview: 'Gazebo is a 3D dynamic simulator...',
      module: 2
    },
    {
      id: 'ch_019',
      title: 'NVIDIA Isaac Overview',
      content_preview: 'NVIDIA Isaac is a comprehensive robotics platform...',
      module: 3
    },
    {
      id: 'ch_029',
      title: 'Vision-Language-Action Models',
      content_preview: 'VLA models represent a significant advancement...',
      module: 4
    }
  ];

  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (searchRef.current && !searchRef.current.contains(event.target as Node)) {
        setShowSuggestions(false);
        setActiveSuggestionIndex(-1);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  useEffect(() => {
    if (query.trim() === '') {
      setSuggestions([]);
      return;
    }

    const filtered = mockSuggestions.filter(suggestion =>
      suggestion.title.toLowerCase().includes(query.toLowerCase()) ||
      suggestion.content_preview.toLowerCase().includes(query.toLowerCase())
    );

    setSuggestions(filtered.slice(0, 5));
    setShowSuggestions(filtered.length > 0);
    setActiveSuggestionIndex(-1);
  }, [query]);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    setQuery(e.target.value);
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      setActiveSuggestionIndex(prev =>
        prev < suggestions.length - 1 ? prev + 1 : prev
      );
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setActiveSuggestionIndex(prev => prev > 0 ? prev - 1 : -1);
    } else if (e.key === 'Enter') {
      e.preventDefault();
      if (activeSuggestionIndex >= 0 && suggestions[activeSuggestionIndex]) {
        setQuery(suggestions[activeSuggestionIndex].title);
        setShowSuggestions(false);
        setActiveSuggestionIndex(-1);
      } else if (query.trim()) {
        handleSearch();
      }
    } else if (e.key === 'Escape') {
      setShowSuggestions(false);
      setActiveSuggestionIndex(-1);
    }
  };

  const handleSuggestionClick = (suggestion: SearchSuggestion) => {
    setQuery(suggestion.title);
    setShowSuggestions(false);
    setActiveSuggestionIndex(-1);
    if (onSearch) {
      onSearch(suggestion.title);
    }
  };

  const handleSearch = () => {
    if (query.trim() && onSearch) {
      onSearch(query.trim());
    }
  };

  return (
    <div className={styles.searchContainer} ref={searchRef}>
      <div className={styles.searchWrapper}>
        <input
          type="text"
          value={query}
          onChange={handleInputChange}
          onKeyDown={handleKeyDown}
          placeholder={placeholder}
          className={styles.searchInput}
          onFocus={() => query && setShowSuggestions(true)}
          aria-label="Search"
        />
        <button
          onClick={handleSearch}
          className={styles.searchButton}
          aria-label="Search"
        >
          🔍
        </button>
      </div>

      {showSuggestions && suggestions.length > 0 && (
        <ul className={styles.suggestionsList}>
          {suggestions.map((suggestion, index) => (
            <li
              key={suggestion.id}
              className={clsx(
                styles.suggestionItem,
                index === activeSuggestionIndex && styles.activeSuggestion
              )}
              onClick={() => handleSuggestionClick(suggestion)}
              onMouseEnter={() => setActiveSuggestionIndex(index)}
            >
              <div className={styles.suggestionTitle}>
                {suggestion.title}
                <span className={styles.moduleTag}>Module {suggestion.module}</span>
              </div>
              <div className={styles.suggestionPreview}>
                {suggestion.content_preview.substring(0, 100)}...
              </div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}

// ✅ Make sure to export default
export default SearchBar;