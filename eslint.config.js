'use strict';

const js = require('@eslint/js');
const globals = require('globals');

module.exports = [
    { ignores: ['node_modules/', 'coverage/', 'docker/data/'] },
    js.configs.recommended,
    {
        files: ['**/*.js'],
        languageOptions: {
            ecmaVersion: 2022,
            sourceType: 'commonjs',
            globals: globals.node
        },
        rules: {
            'no-unused-vars': ['error', { args: 'after-used', argsIgnorePattern: '^_', caughtErrors: 'none' }],
            'no-var': 'error',
            'prefer-const': 'error',
            eqeqeq: ['error', 'always']
        }
    },
    {
        files: ['test/**/*.js'],
        languageOptions: { globals: globals.mocha }
    }
];
