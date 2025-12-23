# Image Optimization Notes

This directory contains optimized images for the AI and Robotics Book Platform. All images should be:

- Compressed to reduce file size
- Served in multiple formats (WebP, AVIF) where possible
- Responsive to different screen sizes
- Lazy-loaded where appropriate

## Recommended Image Sizes

- Logo: 64x64 px (favicon), 192x192 px (social sharing), 512x512 px (high-res displays)
- Diagrams: Max 1200px width for readability
- Photos: Appropriate resolution for their display size

## Optimization Tools

- Use ImageOptim, TinyPNG, or Squoosh for compression
- Consider using next-gen formats like WebP or AVIF
- Implement lazy loading for images below the fold