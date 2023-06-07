importScripts('three.min.js');

function createMeshesFromData(data) {


  const geometry = new THREE.BoxGeometry(0.008, 0.008, 0.008);
  const meshes = [];

  for (const point of data) {
    const color = new THREE.Color(point.r / 255, point.g / 255, point.b / 255);
    const material = new THREE.MeshBasicMaterial({ color });
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(point.x, point.y, point.z);
    meshes.push(mesh);
  }

  return meshes;
}
/*
addEventListener('message', event => {
  const data = JSON.parse(event.data);
  const meshes = createMeshesFromData(data);
  postMessage(meshes);
});*/
onmessage = function(e) {
	console.log("Recieve the script");
	const data = JSON.parse(e.data);
	const meshes = createMeshesFromData(data);
	postMessage(meshes);
	
}

