// ProgressTracker.tsx

import React, { useState, useEffect } from 'react';
import styles from './ProgressTracker.module.css';

type ProgressTrackerProps = {
  chapterId: string;
  chapterTitle: string;
  totalSections: number;
  currentSection: number;
  onProgressUpdate?: (progress: number) => void;
};

export default function ProgressTracker({
  chapterId,
  chapterTitle,
  totalSections,
  currentSection,
  onProgressUpdate
}: ProgressTrackerProps): React.ReactElement {
  const [progress, setProgress] = useState<number>(0);

  useEffect(() => {
    const calculatedProgress = totalSections > 0 ? (currentSection / totalSections) * 100 : 0;
    setProgress(calculatedProgress);

    if (onProgressUpdate) {
      onProgressUpdate(calculatedProgress);
    }
  }, [currentSection, totalSections, onProgressUpdate]);

  return (
    <div className={styles.progressTracker}>
      {/* ... rest of JSX */}
    </div>
  );
}