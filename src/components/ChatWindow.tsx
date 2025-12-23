// src/components/ChatWindow.tsx

import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ChatWindow.module.css';

type Message = {
  id: string;
  text: string;
  sender: 'user' | 'ai';
  timestamp: Date;
  confidence?: number;
};

type ChatWindowProps = {
  isOpen: boolean;
  onClose: () => void;
};

export default function ChatWindow({ isOpen, onClose }: ChatWindowProps): React.ReactElement | null {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: 'Hello! I\'m your AI assistant for the Robotics Book. How can I help you today?',
      sender: 'ai',
      timestamp: new Date(),
      confidence: 1.0
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call backend API
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: inputValue,
          context: null,
          chat_history: []
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();

      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.response || data.text || 'I received your question but could not generate a response.',
        sender: 'ai',
        timestamp: new Date(),
        confidence: data.confidence || 0.85
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error processing your request. Please try again.',
        sender: 'ai',
        timestamp: new Date(),
        confidence: 0.0
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <h3>AI Robotics Assistant</h3>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chat"
        >
          ×
        </button>
      </div>

      <div className={styles.chatMessages}>
        {messages.map((message) => (
          <div
            key={message.id}
            className={clsx(
              styles.message,
              message.sender === 'user' ? styles.userMessage : styles.aiMessage
            )}
          >
            <div className={styles.messageContent}>
              {message.text}
              {message.confidence !== undefined && message.sender === 'ai' && (
                <div className={styles.confidenceIndicator}>
                  Confidence: {(message.confidence * 100).toFixed(0)}%
                </div>
              )}
            </div>
            <div className={styles.messageTime}>
              {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
            </div>
          </div>
        ))}
        {isLoading && (
          <div className={clsx(styles.message, styles.aiMessage)}>
            <div className={styles.typingIndicator}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      <form onSubmit={handleSubmit} className={styles.chatInputForm}>
        <input
          type="text"
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          placeholder="Ask a question about robotics..."
          className={styles.chatInput}
          disabled={isLoading}
        />
        <button
          type="submit"
          className={styles.sendButton}
          disabled={isLoading || !inputValue.trim()}
          aria-label="Send message"
        >
          ➤
        </button>
      </form>
    </div>
  );
}