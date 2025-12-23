import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/blog',
    component: ComponentCreator('/blog', '23c'),
    exact: true
  },
  {
    path: '/blog/archive',
    component: ComponentCreator('/blog/archive', '182'),
    exact: true
  },
  {
    path: '/blog/authors',
    component: ComponentCreator('/blog/authors', '0b7'),
    exact: true
  },
  {
    path: '/blog/platform-features',
    component: ComponentCreator('/blog/platform-features', '42f'),
    exact: true
  },
  {
    path: '/blog/tags',
    component: ComponentCreator('/blog/tags', '287'),
    exact: true
  },
  {
    path: '/blog/tags/ai-chat',
    component: ComponentCreator('/blog/tags/ai-chat', '0b2'),
    exact: true
  },
  {
    path: '/blog/tags/announcement',
    component: ComponentCreator('/blog/tags/announcement', 'd54'),
    exact: true
  },
  {
    path: '/blog/tags/features',
    component: ComponentCreator('/blog/tags/features', '807'),
    exact: true
  },
  {
    path: '/blog/tags/physical-ai',
    component: ComponentCreator('/blog/tags/physical-ai', 'bca'),
    exact: true
  },
  {
    path: '/blog/tags/robotics',
    component: ComponentCreator('/blog/tags/robotics', 'c43'),
    exact: true
  },
  {
    path: '/blog/tags/tutorial',
    component: ComponentCreator('/blog/tags/tutorial', '87b'),
    exact: true
  },
  {
    path: '/blog/welcome',
    component: ComponentCreator('/blog/welcome', '6f2'),
    exact: true
  },
  {
    path: '/chat/',
    component: ComponentCreator('/chat/', 'c74'),
    exact: true
  },
  {
    path: '/search/',
    component: ComponentCreator('/search/', '21b'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'f3c'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '14c'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', 'b5b'),
            routes: [
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', 'cda'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter-1-introduction-to-ros2',
                component: ComponentCreator('/docs/module1/chapter-1-introduction-to-ros2', '1ff'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/chapter-2-nodes-and-topics',
                component: ComponentCreator('/docs/module1/chapter-2-nodes-and-topics', 'e23'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module1/chapter01',
                component: ComponentCreator('/docs/module1/chapter01', '995'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter02',
                component: ComponentCreator('/docs/module1/chapter02', 'a1a'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter03',
                component: ComponentCreator('/docs/module1/chapter03', '560'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter04',
                component: ComponentCreator('/docs/module1/chapter04', 'ee1'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter05',
                component: ComponentCreator('/docs/module1/chapter05', '5a8'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter06',
                component: ComponentCreator('/docs/module1/chapter06', '243'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter07',
                component: ComponentCreator('/docs/module1/chapter07', 'c79'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter08',
                component: ComponentCreator('/docs/module1/chapter08', '1f4'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter09',
                component: ComponentCreator('/docs/module1/chapter09', '106'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module1/chapter10',
                component: ComponentCreator('/docs/module1/chapter10', 'b4a'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter-13-introduction-to-gazebo',
                component: ComponentCreator('/docs/module2/chapter-13-introduction-to-gazebo', 'c31'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module2/chapter11',
                component: ComponentCreator('/docs/module2/chapter11', '392'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter12',
                component: ComponentCreator('/docs/module2/chapter12', '4ea'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter13',
                component: ComponentCreator('/docs/module2/chapter13', '416'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter14',
                component: ComponentCreator('/docs/module2/chapter14', 'edc'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter15',
                component: ComponentCreator('/docs/module2/chapter15', '2a3'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter16',
                component: ComponentCreator('/docs/module2/chapter16', '867'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter17',
                component: ComponentCreator('/docs/module2/chapter17', 'b22'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module2/chapter18',
                component: ComponentCreator('/docs/module2/chapter18', '1b5'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter-19-nvidia-isaac-overview',
                component: ComponentCreator('/docs/module3/chapter-19-nvidia-isaac-overview', 'ac1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module3/chapter19',
                component: ComponentCreator('/docs/module3/chapter19', 'b39'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter20',
                component: ComponentCreator('/docs/module3/chapter20', 'dba'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter21',
                component: ComponentCreator('/docs/module3/chapter21', '35f'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter22',
                component: ComponentCreator('/docs/module3/chapter22', '6e6'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter23',
                component: ComponentCreator('/docs/module3/chapter23', 'd01'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter24',
                component: ComponentCreator('/docs/module3/chapter24', '2bd'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter25',
                component: ComponentCreator('/docs/module3/chapter25', '587'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter26',
                component: ComponentCreator('/docs/module3/chapter26', '366'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter27',
                component: ComponentCreator('/docs/module3/chapter27', 'e01'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module3/chapter28',
                component: ComponentCreator('/docs/module3/chapter28', '410'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter-29-vision-language-action-models',
                component: ComponentCreator('/docs/module4/chapter-29-vision-language-action-models', '465'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module4/chapter29',
                component: ComponentCreator('/docs/module4/chapter29', 'e54'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter30',
                component: ComponentCreator('/docs/module4/chapter30', 'a5c'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter31',
                component: ComponentCreator('/docs/module4/chapter31', '2af'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter32',
                component: ComponentCreator('/docs/module4/chapter32', 'e1a'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter33',
                component: ComponentCreator('/docs/module4/chapter33', '19c'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter34',
                component: ComponentCreator('/docs/module4/chapter34', '100'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter35',
                component: ComponentCreator('/docs/module4/chapter35', '357'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter36',
                component: ComponentCreator('/docs/module4/chapter36', 'e8b'),
                exact: true,
                sidebar: "bookSidebar"
              },
              {
                path: '/docs/module4/chapter37',
                component: ComponentCreator('/docs/module4/chapter37', '499'),
                exact: true,
                sidebar: "bookSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', 'e5f'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
