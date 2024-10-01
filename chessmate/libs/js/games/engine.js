var engine = 
{
	receive 			:	function ( message ) 
							{
								var data = message.data;
								
								if (engine[data.event]) engine[data.event](data);
							},

	send 				: 	function ( type , data , buffers)
							{
								data = data || {};
								data.event = type;
								
								try { self.postMessage ( data , buffers ) } catch (e) {};
							},

	init				:	function (properties)
							{
								engine.send('init', {test:'aloha'});
							},
							
	findShortestPath	:	function (data)	
							{
								//console.log('WORKER.findShortestPath', data);
								
								var paths = data.paths;
								var start = data.start;
								var end = data.end;
								
								var graph = new DirectedGraph();
								
								for (var path in paths)
								{
									graph.addVertex(path, paths[path]);
								};
								
								var result = graph.findShortestPathsFrom(start);
								
								
								
								engine.send('path', {path:result[end], id:data.id});
							}


}

self.onmessage = engine.receive;


function DirectedGraph()
{
	this.vertices = {};
	
	this.addVertex = function(name, cost)
	{
		this.vertices[name] = cost;
	}
	
	this.findShortestPathsFrom = function (startVertex) 
	{ 
		var graph = this;
		var dist = {};
		var prev = {};
		var q = {};
		var shortestPaths = {};
		
		for (var vertex in graph.vertices) 
		{
			dist[vertex] = Infinity;
			prev[vertex] = null;
			q[vertex] = graph.vertices[vertex];
			shortestPaths[vertex] = [];
		}
		
		dist[startVertex] = 0;
		
		while (Object.keys(q).length !== 0) 
		{
			var smallest = findSmallest(dist, q);
			var smallestNode = graph.vertices[smallest] 
			
			// search for the vertex u in the vertex set Q that has the least dist[smallest] value.
			for (var neighbor in smallestNode) 
			{
				var alt = dist[smallest] + smallestNode[neighbor];
				
				// smallestNode[neighbor] is the distance between smallest and neighbor
				if (alt < dist[neighbor]) 
				{
					dist[neighbor] = alt
					prev[neighbor] = smallest
				}
			}
		}
		
		getShortestPaths(prev, shortestPaths, startVertex, dist)
	  
		function findSmallest(dist, q) 
		{ 
			var min = Infinity;
			var minNode;
		
			for (var node in q) 
			{
				if (dist[node] <= min) 
				{
					min = dist[node]
					minNode = node;
				}
			}
		
			delete q[minNode]
			return minNode;
		}				  
		
		function getShortestPaths(previous, shortestPaths, startVertex, dist) 
		{ 
			for (var node in shortestPaths) {
			var path = shortestPaths[node];
		
			while(previous[node]) 
			{
				path.push(node);
				node = previous[node];
			}
		
			// get the starting node in there as well if there was a path from it
			if (dist[node] === 0) 
			{
				path.push(node);
			} 
			path.reverse();
		  }
		}
		
		return 	shortestPaths;
	}	
}
