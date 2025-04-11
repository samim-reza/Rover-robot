import React, { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';

const StlViewer = () => {
  const mountRef = useRef();

  useEffect(() => {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf0f0f0);

    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    camera.position.set(120, 270, 450); // Move the camera further out

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true; // Enable shadows
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    mountRef.current.appendChild(renderer.domElement);

    const ambientLight = new THREE.AmbientLight(0x404040);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 2); // Increase intensity
    directionalLight.position.set(1, 1, 1).normalize();
    directionalLight.castShadow = true; // Enable shadow
    scene.add(directionalLight);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.update();

    const loader = new STLLoader();
    loader.load(
      'http://localhost:5000/stl/rover',
      (geometry) => {
        console.log('STL loaded successfully');
        const material = new THREE.MeshStandardMaterial({ color: 0xff5533 });
        const mesh = new THREE.Mesh(geometry, material);
        mesh.rotation.y = -Math.PI / 2; // Rotate the model

        // Scale and position adjustments
        mesh.scale.set(0.1, 0.1, 0.1); // Adjusted scale to 0.1 for smaller size
        mesh.position.set(50, 150, 0);    // Center the model at the origin
        mesh.castShadow = true;        // Enable shadow casting
        mesh.receiveShadow = true;     // Enable shadow receiving

        scene.add(mesh);
      },
      undefined,
      (err) => {
        console.error('Error loading STL file:', err);
      }
    );

    // Add helpers for debugging
    const gridHelper = new THREE.GridHelper(10, 10);
    scene.add(gridHelper);

    // Create axes helper and position it correctly
    const axisHelper = new THREE.AxesHelper(5);
    scene.add(axisHelper);

    // Adjusting the rotation of the axis helper
    axisHelper.rotation.x = -Math.PI / 2; // Apply rotation to the axis helper
    axisHelper.position.set(0, 0, 0); // Position the axis helper

    // Optional: Set the axis helper's size to fit better
    axisHelper.scale.set(1, 1, 1);

    const animate = () => {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    };
    animate();

    return () => {
      renderer.dispose();
      if (mountRef.current) mountRef.current.innerHTML = '';
    };
  }, []);

  return <div ref={mountRef} style={{ width: '100%', height: '100vh' }} />;
};

export default StlViewer;
