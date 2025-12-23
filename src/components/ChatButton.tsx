// src/components/ChatButton.tsx

import React, { useState } from 'react';
import styles from './ChatButton.module.css';

type ChatButtonProps = {
  onClick: () => void;
};

export default function ChatButton({ onClick }: ChatButtonProps): React.ReactElement | null {
  const [isVisible, setIsVisible] = useState(true);

  // Hide button when chat is open (this would be managed by parent component in a full implementation)
  if (!isVisible) {
    return null;
  }

  return (
    <button
      className={styles.chatButton}
      onClick={onClick}
      aria-label="Open chat"
    >
      <div className={styles.chatIcon}>
        💬
      </div>
      <span className={styles.chatText}>AI Assistant</span>
    </button>
  );
}